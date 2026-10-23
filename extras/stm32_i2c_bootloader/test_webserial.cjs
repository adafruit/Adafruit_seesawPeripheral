// Run with: node extras/stm32_i2c_bootloader/test_webserial.cjs
// No hardware access: file parsing and protocol simulation only.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, 'webserial.html'), 'utf8');
assert.ok(!html.includes('showSaveFilePicker'));
assert.ok(!html.includes('type="checkbox"'));
assert.ok(!html.includes('id="backup"'));
const elements = new Map();
const context = vm.createContext({
  Uint8Array, DataView, TextEncoder, TextDecoder, Blob, URL, setTimeout, clearTimeout,
  navigator: {serial: {}}, window: {isSecureContext: true, addEventListener() {}},
  document: {getElementById(id) {
    if (!elements.has(id)) elements.set(id, {textContent: '', value: '', classList: {add() {}, remove() {}, toggle() {}}, addEventListener() {}});
    return elements.get(id);
  }}
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
const {parseHex, validateImage, Bootloader, SerialBridge} = vm.runInContext('({parseHex, validateImage, Bootloader, SerialBridge})', context);
const base = 0x08000000;
const image = new Uint8Array(57).fill(0x55);
new DataView(image.buffer).setUint32(0, 0x20001800, true);
new DataView(image.buffer).setUint32(4, base + 9, true);
function record(type, address, payload = []) {
  const bytes = [payload.length, address >> 8, address & 255, type, ...payload];
  bytes.push((-bytes.reduce((a, b) => a + b, 0)) & 255);
  return ':' + bytes.map(v => v.toString(16).padStart(2, '0')).join('').toUpperCase();
}
const header = record(4, 0, [8, 0]), data = record(0, 0, [...image]), eof = record(1, 0);
const validHex = [header, data, eof].join('\n');
assert.deepEqual(validateImage(image), image);
assert.deepEqual(parseHex(validHex), image);
assert.deepEqual(parseHex([record(2, 0, [0, 0]), header, data, record(5, 0, [8, 0, 0, 9]), eof].join('\r\n')), image);
assert.throws(() => parseHex(validHex.replace(data, data.slice(0, -2) + '00')), /checksum/);
assert.throws(() => parseHex([header, data].join('\n')), /EOF/);
assert.throws(() => parseHex([header, data, data, eof].join('\n')), /Overlapping/);
assert.throws(() => parseHex([data, eof].join('\n')), /outside/);
assert.throws(() => parseHex([header, data, record(0, 30720, [0]), eof].join('\n')), /outside/);
assert.throws(() => parseHex([record(4, 0, [0x1f, 0xff]), record(0, 0x7800, [0]), eof].join('\n')), /outside/);
assert.throws(() => parseHex([header, data, eof, data].join('\n')), /Invalid/);
assert.throws(() => validateImage(new Uint8Array(32768)), /30720/);
assert.throws(() => validateImage(new Uint8Array(8)), /stack/);
const badEntry = image.slice(); new DataView(badEntry.buffer).setUint32(4, base + 8, true);
assert.throws(() => validateImage(badEntry), /reset vector/);
const sparse = parseHex([header, record(0, 0, [...image.slice(0, 16)]), record(0, 32, [0xAA]), eof].join('\n'));
assert.equal(sparse.length, 33); assert.equal(sparse[16], 255); assert.equal(sparse[32], 0xAA);
assert.throws(() => parseHex([header, record(0, 0, [...image.slice(0, 8)]), record(0, 32, [0]), eof].join('\n')), /gap/);

// Simulate the exact W/U transport boundary, including all AN4221 packets.
class ROM {
  constructor() { this.memory = new Uint8Array(32768).fill(255); this.memory.fill(0x5A, 30720); this.options = new Uint8Array(128).fill(0xAA); this.reads = []; this.goRequested = false; this.queue = []; this.state = 'command'; this.writes = 0; this.erases = []; this.failWrite = false; }
  async close() { this.closed = true; }
  async command(operation, values) {
    assert.equal(values[0], 0x64);
    if (operation === 'U') {
      const bytes = this.queue.shift(); assert.ok(bytes, 'Unexpected ROM read'); assert.equal(bytes.length, values[1]);
      return 'DATA ' + bytes.map(v => v.toString(16).padStart(2, '0')).join(' ').toUpperCase();
    }
    assert.equal(operation, 'W'); const bytes = values.slice(1);
    assert.ok(bytes.length <= 32); const ack = () => this.queue.push([0x79]);
    if (this.state === 'command') {
      assert.equal(bytes[1], bytes[0] ^ 255); this.code = bytes[0]; ack();
      if (this.code === 2) { this.queue.push([1, 4, 0x43]); ack(); }
      else if (this.code === 0) { this.queue.push([18, 0x11, 0x11, 0x21, 0x32, 0x45, ...Array(14).fill(0)]); ack(); }
      else this.state = this.code === 0x45 ? 'page-count' : 'address';
    } else if (this.state === 'address') {
      assert.equal(bytes.reduce((a, b) => a ^ b, 0), 0);
      this.address = bytes.slice(0, 4).reduce((a, b) => a * 256 + b, 0); ack();
      if (this.code === 0x21) this.goRequested = true;
      this.state = this.code === 0x21 ? 'command' : 'data';
    } else if (this.state === 'page-count') {
      assert.equal(bytes.reduce((a, b) => a ^ b, 0), 0); this.pages = bytes[0] * 256 + bytes[1] + 1; this.state = 'pages'; ack();
    } else if (this.state === 'pages') {
      assert.equal(bytes.length, this.pages * 2 + 1); assert.equal(bytes.reduce((a, b) => a ^ b, 0), 0);
      for (let i = 0; i < this.pages; i++) { const page = bytes[i * 2] * 256 + bytes[i * 2 + 1]; assert.ok(page < 15); this.erases.push(page); this.memory.fill(255, page * 2048, (page + 1) * 2048); }
      this.state = 'command'; ack();
    } else if (this.code === 0x11) {
      assert.equal(bytes[1], bytes[0] ^ 255); ack();
      this.reads.push({address: this.address, count: bytes[0] + 1, beforeErase: this.erases.length === 0});
      const optionRead = this.address >= 0x1FFF7800;
      const memory = optionRead ? this.options : this.memory;
      const offset = this.address - (optionRead ? 0x1FFF7800 : base);
      this.queue.push([...memory.slice(offset, offset + bytes[0] + 1)]); this.state = 'command';
    } else {
      assert.equal(this.code, 0x32); assert.equal(bytes.reduce((a, b) => a ^ b, 0), 0);
      const block = bytes.slice(1, -1); assert.equal(block.length, bytes[0] + 1); assert.equal(block.length % 8, 0); assert.equal(this.address % 8, 0);
      if (this.failWrite) throw new Error('injected write failure');
      assert.ok(this.address >= base && this.address + block.length <= base + 30720);
      this.memory.set(block, this.address - base); this.writes++; this.state = 'command'; ack();
    }
    return 'OK';
  }
}
(async () => {
  const device = new ROM(), rom = new Bootloader(device);
  await rom.identify(); await rom.erase(image.length); const padded = await rom.program(image);
  assert.equal(padded.length, 64); assert.equal(device.writes, 3);
  assert.deepEqual(await rom.readMemory(base, padded.length), padded);
  assert.ok(device.memory.slice(30720).every(v => v === 0x5A));
  await rom.go(); assert.deepEqual(device.erases, [0]);
  await assert.rejects(() => rom.erase(32768), /EEPROM/);
  const failed = new ROM(); failed.failWrite = true;
  await assert.rejects(() => new Bootloader(failed).program(image), /injected/);
  assert.equal(failed.writes, 0);
  const bridge = new SerialBridge({});
  bridge.writer = {write: async () => { bridge.pending.resolve('OK'); bridge.pending = null; }};
  assert.equal(await bridge.command('W', [0x64, 0, 255]), 'OK');
  bridge.lines.push('OK'); await assert.rejects(() => bridge.command('W', [0x64, 0, 255]), /stale/);
  // Exercise the Upload button path without a checkbox or file-save API.
  const direct = new ROM(); context.testDevice = direct; context.testImage = image;
  vm.runInContext('bridge = testDevice; firmware = {image: testImage}; controls();', context);
  assert.equal(elements.get('upload').disabled, false);
  await vm.runInContext('upload()', context);
  assert.equal(direct.goRequested, true); assert.equal(direct.closed, true);
  assert.ok(elements.get('status').textContent.startsWith('Upload verified.'));
  assert.ok(!direct.reads.some(r => r.beforeErase && r.address >= base && r.address < base + 30720), 'No application backup reads before erase');
  assert.ok(direct.memory.slice(30720).every(v => v === 0x5A));
  assert.ok(direct.options.every(v => v === 0xAA));
  assert.ok(!elements.has('confirm') && !elements.has('backup'));
  const directFailure = new ROM(); directFailure.failWrite = true; context.testDevice = directFailure;
  vm.runInContext('bridge = testDevice;', context);
  await vm.runInContext('upload()', context);
  assert.equal(directFailure.goRequested, false); assert.equal(directFailure.closed, true);
  assert.ok(elements.get('status').textContent.startsWith('Stopped.'));
  process.stdout.write('PASS: HEX/BIN validation, ROM packets, EEPROM exclusion, serial framing, direct upload without backup/checkbox, and failure stop before Go.\n');
})().catch(error => { console.error(error); process.exitCode = 1; });

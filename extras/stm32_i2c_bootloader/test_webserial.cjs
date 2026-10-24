// Run with: node extras/stm32_i2c_bootloader/test_webserial.cjs
// No hardware access: file parsing and protocol simulation only.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, 'webserial.html'), 'utf8');
assert.ok(!html.includes('showSaveFilePicker'));
assert.ok(!html.includes('id="confirm"'));
assert.ok(html.includes('id="verifyAfterFlash" checked'));
assert.ok(!html.includes('id="backup"'));
assert.ok(!html.includes('<footer>'));
assert.ok(!html.includes('id="entry"') && !html.includes('id="address"'));
const elements = new Map();
const windowEvents = new Map();
let downloadedBlob;
const context = vm.createContext({
  Uint8Array, DataView, TextEncoder, TextDecoder, Blob, setTimeout, clearTimeout,
  URL: {createObjectURL(blob) { downloadedBlob = blob; return 'blob:readout'; }, revokeObjectURL() {}},
  navigator: {serial: {}}, window: {isSecureContext: true, addEventListener(name, handler) { windowEvents.set(name, handler); }},
  document: {getElementById(id) {
    if (!elements.has(id)) elements.set(id, {textContent: '', value: '', classList: {add() {}, remove() {}, toggle() {}}, addEventListener() {}, click() { this.clicked = true; }});
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
  // Write uses default-on verification, with no confirmation or file-save API.
  const direct = new ROM(); context.testDevice = direct; context.testImage = image;
  context.document.getElementById('verifyAfterFlash').checked = true;
  vm.runInContext('bridge = testDevice; romReady = true; firmware = {image: testImage}; controls();', context);
  assert.equal(elements.get('write').disabled, false);
  await vm.runInContext('runOperation("write")', context);
  assert.equal(direct.goRequested, true); assert.equal(direct.closed, true);
  assert.ok(elements.get('status').textContent.startsWith('Write verified.'));
  assert.ok(!direct.reads.some(r => r.beforeErase && r.address >= base && r.address < base + 30720), 'No application backup reads before erase');
  assert.ok(direct.memory.slice(30720).every(v => v === 0x5A));
  assert.ok(direct.options.every(v => v === 0xAA));
  assert.ok(!elements.has('confirm') && !elements.has('backup'));
  const directFailure = new ROM(); directFailure.failWrite = true; context.testDevice = directFailure;
  vm.runInContext('bridge = testDevice; romReady = true;', context);
  await vm.runInContext('runOperation("write")', context);
  assert.equal(directFailure.goRequested, false); assert.equal(directFailure.closed, true);
  assert.ok(elements.get('status').textContent.startsWith('Stopped.'));
  // Unchecked verification skips only application readback after Write.
  const unchecked = new ROM(); context.testDevice = unchecked;
  context.document.getElementById('verifyAfterFlash').checked = false;
  vm.runInContext('bridge = testDevice; romReady = true;', context);
  await vm.runInContext('runOperation("write")', context);
  assert.equal(unchecked.goRequested, true);
  assert.ok(!unchecked.reads.some(r => r.address >= base && r.address < base + 30720));
  assert.ok(elements.get('status').textContent.startsWith('Write complete (not verified).'));
  // Verify is read-only, including when Verify after flash is unchecked.
  const verifyDevice = new ROM(); verifyDevice.memory.set(padded); context.testDevice = verifyDevice;
  vm.runInContext('bridge = testDevice; romReady = true;', context);
  await vm.runInContext('runOperation("verify")', context);
  assert.equal(verifyDevice.writes, 0); assert.equal(verifyDevice.erases.length, 0);
  assert.equal(verifyDevice.goRequested, false);
  assert.ok(elements.get('status').textContent.startsWith('Verify passed.'));
  const mismatch = new ROM(); mismatch.memory.set(padded); mismatch.memory[16] ^= 1; context.testDevice = mismatch;
  vm.runInContext('bridge = testDevice; romReady = true;', context);
  await vm.runInContext('runOperation("verify")', context);
  assert.ok(elements.get('status').textContent.startsWith('Stopped.'));
  assert.equal(mismatch.writes, 0); assert.equal(mismatch.erases.length, 0);
  assert.equal(mismatch.goRequested, false);
  // Read needs no input file and downloads every flash byte, including EEPROM.
  const readDevice = new ROM(); context.testDevice = readDevice;
  vm.runInContext('bridge = testDevice; romReady = true; firmware = null; controls();', context);
  assert.equal(elements.get('read').disabled, false);
  assert.equal(elements.get('write').disabled, true); assert.equal(elements.get('verify').disabled, true);
  await vm.runInContext('runOperation("read")', context);
  assert.deepEqual(new Uint8Array(await downloadedBlob.arrayBuffer()), readDevice.memory);
  assert.equal(elements.get('readout').clicked, true);
  assert.equal(readDevice.writes, 0); assert.equal(readDevice.erases.length, 0);
  assert.equal(readDevice.goRequested, false); assert.equal(readDevice.closed, true);
  // O replies are framed by a serial-only END command, including optional lines.
  const optionDump = 'OPTIONS' + ' AA'.repeat(128);
  const summary = 'nBOOT_SEL=0 nBOOT1=1 nBOOT0=1 BOOT_LOCK=0 RDP=0xAA';
  const verified = 'VERIFIED boot options and all other defined settings; complements valid';
  const ack = 'ACK option write; readback still required';
  const optionBridge = new SerialBridge({});
  const serialWrites = [];
  optionBridge.writer = {write: async bytes => {
    serialWrites.push(new TextDecoder().decode(bytes));
    for (const line of [optionDump, summary, verified, 'ERR operation']) optionBridge.pending.resolve(line);
  }};
  assert.deepEqual(Array.from(await optionBridge.command('O', [0x64])), [optionDump, summary, verified]);
  assert.equal(serialWrites[0], 'O 64\nEND\n');
  assert.equal(optionBridge.pending, null); assert.equal(optionBridge.lines.length, 0);
  // Exercise the real reader with fragmented LF/CR output and the boundary in
  // the same chunk as the last response, not only the command's resolver.
  const streamBridge = new SerialBridge({});
  let finishRead;
  const chunks = ['Adafruit STM32C011 I2C bootloader bridge READY\n\r', optionDump.slice(0, 99), optionDump.slice(99) + '\n\r' + summary + '\n', '\rERR operation\n\r'];
  streamBridge.reader = {read: async () => chunks.length
    ? {value: new TextEncoder().encode(chunks.shift()), done: false}
    : new Promise(resolve => { finishRead = resolve; })};
  streamBridge.writer = {write: async () => { streamBridge.reading = streamBridge.readLoop(); }};
  assert.deepEqual(Array.from(await streamBridge.command('O', [0x64])), [optionDump, summary]);
  assert.equal(streamBridge.lines.length, 0);
  finishRead({done: true}); await streamBridge.reading;

  class OptionBridge {
    constructor(replies) { this.replies = replies; this.commands = []; }
    async command(operation, values) { this.commands.push([operation, ...values]); return this.replies; }
    async close() { this.closed = true; }
  }
  const installOptions = replies => {
    const device = new OptionBridge(replies); context.testDevice = device;
    vm.runInContext('bridge = testDevice; romReady = true; firmware = null; optionPending = false; optionStopped = false; targetResetConfirmed = false;', context);
    return device;
  };
  const query = installOptions([optionDump, summary]);
  vm.runInContext('controls()', context);
  assert.equal(elements.get('readOptions').disabled, false);
  assert.equal(elements.get('enableBoot0').disabled, true);
  await vm.runInContext('runBootOptions()', context);
  assert.deepEqual(query.commands.map(command => Array.from(command)), [['O', 0x64]]);
  assert.equal(query.closed, undefined);
  assert.equal(elements.get('bootOptions').textContent, summary);
  assert.match(elements.get('optionStatus').textContent, /no option write/);

  const enable = installOptions([optionDump, summary, ack, 'Keep Metro powered. Reset/power-cycle C011 before another option write', 'Re-enter ROM, then query O 64 (or O 49 from running seesaw) to verify']);
  await vm.runInContext('runBootOptions("boot0")', context);
  assert.equal(enable.commands.length, 0, 'Reset acknowledgement is required');
  vm.runInContext('targetResetConfirmed = true;', context);
  await vm.runInContext('runBootOptions("boot0")', context);
  assert.deepEqual(Array.from(enable.commands[0]), ['O', 0x64, 0, 1, 1]);
  assert.equal(vm.runInContext('targetResetConfirmed', context), false);
  assert.match(elements.get('optionStatus').textContent, /NOT verified/);
  assert.equal(enable.closed, undefined);
  assert.equal(elements.get('write').disabled, true);
  assert.equal(elements.get('enableBoot0').disabled, true);
  assert.equal(elements.get('readOptions').disabled, true, 'Rediscover ROM after an option write');
  let warnedBeforeClosing = false;
  windowEvents.get('beforeunload')({preventDefault() { warnedBeforeClosing = true; }});
  assert.equal(warnedBeforeClosing, true);
  await vm.runInContext('runBootOptions("flash"); runOperation("read");', context);
  assert.equal(enable.commands.length, 1, 'No write or unrelated operation during pending verification');
  enable.replies = [optionDump, summary, verified];
  vm.runInContext('romReady = true;', context);
  await vm.runInContext('runBootOptions()', context);
  assert.deepEqual(Array.from(enable.commands[1]), ['O', 0x64]);
  assert.match(elements.get('optionStatus').textContent, /^Verified:/);
  assert.equal(vm.runInContext('optionPending', context), false);

  const flash = installOptions([optionDump, summary, 'OK boot options already match; no write']);
  vm.runInContext('targetResetConfirmed = true;', context);
  await vm.runInContext('runBootOptions("flash")', context);
  assert.deepEqual(Array.from(flash.commands[0]), ['O', 0x64, 1, 1, 1]);
  assert.equal(vm.runInContext('optionPending', context), false);
  assert.match(elements.get('optionStatus').textContent, /no write was needed/);

  for (const replies of [
    ['ERR command'], [optionDump, summary, 'ERR requires RDP level 0 and BOOT_LOCK=0; refusing write'],
    ['ERR C011 ROM entry/identification failed'],
    [optionDump, summary, 'ERR option write unconfirmed; do not retry'],
    [optionDump, summary, 'ERR pending option write does not match; no further writes'],
    [optionDump], [optionDump, summary]
  ]) {
    const failure = installOptions(replies); vm.runInContext('targetResetConfirmed = true;', context);
    await vm.runInContext('runBootOptions("boot0")', context);
    assert.equal(failure.commands.length, 1);
    assert.equal(failure.closed, undefined);
    assert.equal(vm.runInContext('optionStopped', context), true);
    assert.equal(elements.get('enableBoot0').disabled, true);
    assert.match(elements.get('optionStatus').textContent, /Stopped; no automatic retry/);
    if (replies.includes('ERR C011 ROM entry/identification failed')) {
      assert.equal(vm.runInContext('optionPending', context), false, 'No write occurred, so there is no pending snapshot');
      failure.replies = [optionDump, summary];
      vm.runInContext('romReady = true;', context);
      await vm.runInContext('runBootOptions()', context);
      assert.equal(vm.runInContext('optionStopped', context), false);
    }
  }
  const lostSnapshot = installOptions([optionDump, summary]);
  vm.runInContext('optionPending = true;', context);
  await vm.runInContext('runBootOptions()', context);
  assert.equal(vm.runInContext('optionPending', context), true);
  assert.match(elements.get('optionStatus').textContent, /may have reset/);
  let confirmations = 0;
  context.window.confirm = () => { confirmations++; return false; };
  await vm.runInContext('connect()', context);
  assert.equal(confirmations, 1); assert.equal(lostSnapshot.closed, undefined);
  context.window.confirm = () => true;
  await vm.runInContext('connect()', context);
  assert.equal(lostSnapshot.closed, true);
  assert.match(elements.get('optionStatus').textContent, /without verification/);
  // Discovery exercises the real ROM protocol simulator, not an address-only
  // success stub. Scanning never sends the boot key or changes flash/options.
  class DiscoveryBridge extends ROM {
    constructor(addresses = []) { super(); this.addresses = addresses; this.commands = []; this.ids = new Map(); this.bootWorks = true; this.romValid = true; this.romActive = addresses.includes(0x64); }
    async command(operation, values) {
      this.commands.push([operation, ...values]);
      if (operation === 'S') return 'SCAN' + this.addresses.map(address => ' ' + address.toString(16).toUpperCase().padStart(2, '0')).join('');
      if (operation === 'R') {
        assert.notEqual(values[0], 0x64, 'Never send seesaw register queries to ROM');
        return this.ids.get(values[0]) || 'DATA 55';
      }
      if (operation === 'W' && values[0] !== 0x64) {
        assert.deepEqual(Array.from(values.slice(1)), [0, 0x7e, 0xb0, 7, 0x10, 0xad]);
        if (this.bootWorks) { this.addresses = this.addresses.filter(address => address !== values[0]); this.addresses.push(0x64); this.romActive = true; }
        return 'OK';
      }
      if (!this.romActive || !this.romValid) throw new Error('ERR I2C 2');
      if (operation === 'O') return [optionDump, summary, verified];
      return super.command(operation, values);
    }
  }
  const installDiscovery = device => {
    context.testDevice = device;
    vm.runInContext('bridge = testDevice; optionPending = false; optionStopped = false; romReady = false; targetResetConfirmed = false; devices = []; controls();', context);
  };
  const emptyBus = new DiscoveryBridge(); installDiscovery(emptyBus);
  await vm.runInContext('scanDevices()', context);
  assert.match(elements.get('deviceStatus').textContent, /No I²C devices/);
  assert.equal(elements.get('readOptions').disabled, true);
  assert.equal(emptyBus.commands.length, 1);

  const discovered = new DiscoveryBridge([0x49, 0x50]); discovered.ids.set(0x49, 'DATA 90'); installDiscovery(discovered);
  await vm.runInContext('scanDevices()', context);
  assert.equal(elements.get('device').value, String(0x49));
  assert.equal(elements.get('bootRom').disabled, false);
  assert.equal(elements.get('readOptions').disabled, true);
  assert.ok(elements.get('device').innerHTML.includes('Unknown / unsupported'));
  assert.ok(discovered.commands.every(command => command[0] === 'S' || command[0] === 'R'));
  await vm.runInContext('(async () => { await scanDevices(true); await bootIntoROM(); })()', context);
  assert.equal(vm.runInContext('romReady', context), true);
  assert.equal(elements.get('device').value, String(0x64));
  assert.equal(elements.get('enableBoot0').disabled, false);
  assert.equal(discovered.writes, 0); assert.equal(discovered.erases.length, 0);
  assert.equal(discovered.goRequested, false);
  assert.equal(discovered.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length, 1);

  const romOnly = new DiscoveryBridge([0x64]); installDiscovery(romOnly);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('romReady', context), true);
  assert.equal(elements.get('bootRom').disabled, true);
  assert.equal(elements.get('readOptions').disabled, false);
  assert.equal(elements.get('enableBoot0').disabled, true, 'Ordinary scan is not reset confirmation');
  const impostor = new DiscoveryBridge([0x64]); impostor.romValid = false; installDiscovery(impostor);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('romReady', context), false);
  assert.match(elements.get('deviceStatus').textContent, /no supported C011/);

  const multiple = new DiscoveryBridge([0x49, 0x4a]); multiple.ids.set(0x49, 'DATA 90'); multiple.ids.set(0x4a, 'DATA 90'); installDiscovery(multiple);
  await vm.runInContext('scanDevices()', context);
  assert.equal(elements.get('device').value, '');
  assert.equal(elements.get('bootRom').disabled, true);
  elements.get('device').value = String(0x4a); vm.runInContext('selectDevice()', context);
  await vm.runInContext('bootIntoROM()', context);
  assert.equal(multiple.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length, 0);
  assert.equal(multiple.commands.filter(command => command[0] === 'W' && command[1] === 0x4a).length, 1);

  const occupied = new DiscoveryBridge([0x49, 0x64]); occupied.ids.set(0x49, 'DATA 90'); installDiscovery(occupied);
  await vm.runInContext('scanDevices()', context);
  elements.get('device').value = String(0x49); vm.runInContext('selectDevice()', context);
  assert.equal(elements.get('bootRom').disabled, true);
  await vm.runInContext('bootIntoROM()', context);
  assert.equal(occupied.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length, 0);
  const newlyOccupied = new DiscoveryBridge([0x49]); newlyOccupied.ids.set(0x49, 'DATA 90'); installDiscovery(newlyOccupied);
  await vm.runInContext('scanDevices()', context);
  newlyOccupied.addresses.push(0x64);
  await vm.runInContext('bootIntoROM()', context);
  assert.match(elements.get('deviceStatus').textContent, /now occupied/);
  assert.equal(newlyOccupied.commands.filter(command => command[0] === 'W').length, 0);

  const noEntry = new DiscoveryBridge([0x49]); noEntry.ids.set(0x49, 'DATA 90'); noEntry.bootWorks = false; installDiscovery(noEntry);
  await vm.runInContext('(async () => { await scanDevices(); await bootIntoROM(); })()', context);
  assert.equal(vm.runInContext('romReady', context), false);
  assert.match(elements.get('deviceStatus').textContent, /may lack boot-entry support/);
  const oldSketch = {command: async () => { throw new Error('ERR command'); }}; installDiscovery(oldSketch);
  await vm.runInContext('scanDevices()', context);
  assert.match(elements.get('deviceStatus').textContent, /Upload the latest/);
  const malformedScan = {command: async () => 'SCAN 49 49'}; installDiscovery(malformedScan);
  await vm.runInContext('scanDevices()', context);
  assert.match(elements.get('deviceStatus').textContent, /Invalid I²C address list/);

  // A target reset/discovery does not close or reset the Metro, so a pending
  // option snapshot can still be checked through the same connection.
  const pendingDevice = new DiscoveryBridge([0x49]); pendingDevice.ids.set(0x49, 'DATA 90'); installDiscovery(pendingDevice);
  vm.runInContext('optionPending = true;', context);
  await vm.runInContext('(async () => { await scanDevices(true); await bootIntoROM(); await runBootOptions(); })()', context);
  assert.equal(pendingDevice.closed, undefined);
  assert.equal(vm.runInContext('optionPending', context), false);
  assert.match(elements.get('optionStatus').textContent, /^Verified:/);
  process.stdout.write('PASS: file/firmware tests; boot-option framing and safety; scan/identify/select; application-to-ROM; ROM-only/unknown/empty buses; address conflicts; old sketch; pending verification after rediscovery.\n');
})().catch(error => { console.error(error); process.exitCode = 1; });

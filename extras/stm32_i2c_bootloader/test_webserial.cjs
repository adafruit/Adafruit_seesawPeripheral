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
assert.ok(!html.includes('id="resetScan"'));
assert.ok(!html.includes('id="target"'));
assert.ok(!html.includes('Connect to detect your C011 or C031.'));
for (const id of ['deviceStep', 'firmwareStep', 'bootOptionsStep']) {
  assert.match(html, new RegExp(`<section id="${id}"[^>]* hidden>`));
}
assert.match(html, /id="scan"[^>]*>Scan<\/button>/);
assert.match(html, /id="connect" class="action"/);
assert.match(html, /id="scan" class="action"/);
assert.match(html, /id="wizardScan" class="action hidden"/);
assert.match(html, /button\[hidden\]\{display:none\}/);
assert.ok(html.indexOf('3. BOOT0 recovery') < html.indexOf('4. Firmware'));
assert.match(html, /Disabling recovery is not recommended/);
const elements = new Map();
const windowEvents = new Map();
let downloadedBlob;
const context = vm.createContext({
  Uint8Array, DataView, TextEncoder, TextDecoder, Blob, setTimeout, clearTimeout,
  URL: {createObjectURL(blob) { downloadedBlob = blob; return 'blob:readout'; }, revokeObjectURL() {}},
  navigator: {serial: {}}, window: {isSecureContext: true, addEventListener(name, handler) { windowEvents.set(name, handler); }},
  document: {getElementById(id) {
    assert.notEqual(id, 'target', 'No target dropdown remains');
    if (!elements.has(id)) elements.set(id, {textContent: '', value: '', open: false, events: {}, classList: {add() {}, remove() {}, toggle(name, enabled) { this[name] = enabled; }}, addEventListener(name, handler) { this.events[name] = handler; }, click() { this.clicked = true; }, showModal() { this.open = true; }, close() { this.open = false; }});
    return elements.get(id);
  }}
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
vm.runInContext('controls()', context);
assert.equal(elements.get('connect').classList.connected, false);
for (const id of ['deviceStep', 'firmwareStep', 'bootOptionsStep']) assert.equal(elements.get(id).hidden, true);
const {parseHex, validateImage, Bootloader, SerialBridge, PROFILES} = vm.runInContext('({parseHex, validateImage, Bootloader, SerialBridge, PROFILES})', context);
assert.equal(PROFILES.c011.getLength, 20);
assert.equal(PROFILES.c031.getLength, 20);
// Captured from C031 device 0x10016453; do not derive this from the profile.
const observedGetFrame = [0x12, 0x11, 0x00, 0x01, 0x02, 0x11, 0x21, 0x31,
  0x44, 0x63, 0x73, 0x82, 0x92, 0x32, 0x45, 0x64, 0x74, 0x83, 0x93, 0xA1];
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
assert.equal(vm.runInContext('target', context), null);
assert.throws(() => validateImage(image), /detect your STM32/);
vm.runInContext('target = PROFILES.c011;', context);
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
  constructor(profile = PROFILES.c011) { this.profile = profile; this.memory = new Uint8Array(32768).fill(255); this.memory.fill(0x5A, 30720); this.options = new Uint8Array(128).fill(0xAA); this.reads = []; this.goRequested = false; this.queue = []; this.state = 'command'; this.writes = 0; this.erases = []; this.failWrite = false; }
  async close() { this.closed = true; }
  async command(operation, values) {
    assert.equal(values[0], this.profile.rom);
    if (operation === 'U') {
      const bytes = this.queue.shift(); assert.ok(bytes, 'Unexpected ROM read'); assert.equal(bytes.length, values[1]);
      return 'DATA ' + bytes.map(v => v.toString(16).padStart(2, '0')).join(' ').toUpperCase();
    }
    assert.equal(operation, 'W'); const bytes = values.slice(1);
    assert.ok(bytes.length <= 32); const ack = () => this.queue.push([0x79]);
    if (this.state === 'command') {
      assert.equal(bytes[1], bytes[0] ^ 255); this.code = bytes[0]; ack();
      if (this.code === 2) { this.queue.push([1, this.profile.id >> 8, this.profile.id & 255]); ack(); }
      else if (this.code === 0) { this.queue.push([...observedGetFrame]); ack(); }
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
      if (this.address === 0x1FFF75A0) {
        this.queue.push([this.flashKiB || 32, 0]); this.state = 'command'; return 'OK';
      }
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
  assert.equal(elements.get('connect').classList.connected, false, 'Clear filled state after automatic disconnect');
  assert.ok(elements.get('status').textContent.startsWith('Write verified.'));
  assert.equal(elements.get('deviceStep').hidden, true);
  assert.equal(elements.get('firmwareStep').hidden, false, 'Keep upload results after automatic disconnect');
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
  assert.equal(elements.get('firmwareStep').hidden, false, 'Keep Save readout accessible after disconnect');
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
    vm.runInContext('bridge = testDevice; romReady = true; firmware = null; bootRecovery = null; optionPending = false; optionStopped = false; targetResetConfirmed = false;', context);
    return device;
  };
  const query = installOptions([optionDump, summary]);
  vm.runInContext('controls()', context);
  assert.equal(elements.get('readOptions').disabled, false);
  assert.equal(elements.get('enableBoot0').disabled, true, 'Read options before offering a boot-mode change');
  await vm.runInContext('runBootOptions()', context);
  assert.deepEqual(query.commands.map(command => Array.from(command)), [['O', 0x64]]);
  assert.equal(query.closed, undefined);
  assert.equal(elements.get('bootOptions').textContent, 'BOOT0 recovery is enabled.');
  assert.equal(elements.get('enableBoot0').hidden, true);
  assert.equal(elements.get('flashBoot').hidden, false);
  assert.equal(elements.get('disableBoot0Warning').hidden, false);
  assert.match(elements.get('optionStatus').textContent, /no option write/);
  const disabledSummary = summary.replace('nBOOT_SEL=0', 'nBOOT_SEL=1');
  const disabledQuery = installOptions([optionDump, disabledSummary]);
  await vm.runInContext('runBootOptions()', context);
  assert.equal(elements.get('enableBoot0').hidden, false);
  assert.equal(elements.get('flashBoot').hidden, true);
  assert.equal(elements.get('disableBoot0Warning').hidden, true);
  assert.deepEqual(disabledQuery.commands.map(command => Array.from(command)), [['O', 0x64]], 'Reading options never sends setter arguments');

  const enable = installOptions([optionDump, summary, ack, 'Keep bridge powered. Reset/power-cycle C011 before another option write', 'Re-enter ROM, then query O 64 (or O 49 from running seesaw) to verify']);
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
  assert.equal(elements.get('enableBoot0').hidden, true);
  assert.equal(elements.get('flashBoot').hidden, true, 'Do not infer the new state before readback');
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
    ['ERR bridge needs a 130-byte I2C transmit buffer; no option write'],
    ['ERR bridge I2C initialization failed; check SDA/SCL pins'],
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
    if (replies.includes('ERR C011 ROM entry/identification failed') || replies.includes('ERR bridge needs a 130-byte I2C transmit buffer; no option write')) {
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
  assert.equal(elements.get('connect').classList.connected, false, 'Clear filled state after manual disconnect');
  assert.match(elements.get('optionStatus').textContent, /without verification/);
  assert.equal(elements.get('bootOptionsStep').hidden, false, 'Keep pending verification visible');
  // Discovery exercises the real ROM protocol simulator, not an address-only
  // success stub. Scanning never sends the boot key or changes flash/options.
  class DiscoveryBridge extends ROM {
    constructor(addresses = [], profile = PROFILES.c011) { super(profile); this.addresses = addresses; this.commands = []; this.ids = new Map(); this.bootWorks = true; this.romValid = true; this.romActive = addresses.includes(this.profile.rom); this.optionReplies = [optionDump, summary, verified]; }
    async command(operation, values) {
      this.commands.push([operation, ...values]);
      if (operation === 'S') return 'SCAN' + this.addresses.map(address => ' ' + address.toString(16).toUpperCase().padStart(2, '0')).join('');
      if (operation === 'R') {
        assert.ok(values[0] !== 0x63 && values[0] !== 0x64, 'Never send seesaw register queries to ROM');
        return this.ids.get(values[0]) || 'DATA 55';
      }
      if ([0x63, 0x64].includes(values[0]) && values[0] !== this.profile.rom) throw new Error('ERR I2C 2');
      if (operation === 'W' && values[0] !== this.profile.rom) {
        assert.deepEqual(Array.from(values.slice(1)), [0, 0x7e, 0xb0, 7, 0x10, 0xad]);
        if (this.bootWorks) { this.addresses = this.addresses.filter(address => address !== values[0]); this.addresses.push(this.profile.rom); this.romActive = true; }
        return 'OK';
      }
      if (!this.romActive || !this.romValid) throw new Error('ERR I2C 2');
      if (operation === 'O') return this.optionReplies;
      return super.command(operation, values);
    }
  }
  const installDiscovery = device => {
    context.testDevice = device;
    vm.runInContext('bridge = testDevice; optionPending = false; optionStopped = false; romReady = false; targetResetConfirmed = false; devices = []; controls();', context);
  };
  const emptyBus = new DiscoveryBridge(); installDiscovery(emptyBus);
  vm.runInContext('target = null; firmwareResult = false;', context);
  await vm.runInContext('scanDevices()', context);
  assert.match(elements.get('deviceStatus').textContent, /No I²C devices/);
  assert.equal(elements.get('readOptions').disabled, true);
  assert.equal(emptyBus.commands.length, 1);
  assert.equal(elements.get('connect').classList.connected, true, 'Connection state does not require a detected STM32');
  assert.equal(elements.get('file').disabled, true);
  assert.equal(elements.get('deviceStep').hidden, false);
  assert.equal(elements.get('firmwareStep').hidden, true);
  assert.equal(elements.get('bootOptionsStep').hidden, true);
  await vm.runInContext('chooseFile({name: "early.bin"})', context);
  assert.match(elements.get('status').textContent, /detect your STM32/);

  const discovered = new DiscoveryBridge([0x49, 0x50]); discovered.ids.set(0x49, 'DATA 90'); installDiscovery(discovered);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('target.name', context), 'C011');
  assert.equal(elements.get('firmwareStep').hidden, false);
  assert.equal(elements.get('bootOptionsStep').hidden, false);
  assert.equal(elements.get('device').value, String(0x49));
  assert.equal(elements.get('bootRom').disabled, false);
  assert.equal(elements.get('bootRom').hidden, false, 'Application still needs the bootloader action');
  assert.equal(elements.get('readOptions').disabled, true);
  assert.ok(elements.get('device').innerHTML.includes('Unknown / unsupported'));
  assert.ok(discovered.commands.every(command => command[0] === 'S' || command[0] === 'R'));
  await vm.runInContext('(async () => { await scanDevices(true); await bootIntoROM(); })()', context);
  assert.equal(vm.runInContext('romReady', context), true);
  assert.equal(elements.get('device').value, String(0x64));
  assert.equal(elements.get('enableBoot0').disabled, false);
  assert.equal(discovered.writes, 0); assert.equal(discovered.erases.length, 0);
  assert.equal(discovered.goRequested, false);
  assert.deepEqual(discovered.commands.filter(command => command[0] === 'O'), [['O', 0x64]], 'ROM entry automatically reads options once');
  assert.equal(discovered.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length, 1);

  const romOnly = new DiscoveryBridge([0x64]); installDiscovery(romOnly);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('romReady', context), true);
  assert.equal(elements.get('bootRom').disabled, true);
  assert.equal(elements.get('bootRom').hidden, true, 'ROM does not need the bootloader action');
  assert.equal(elements.get('readOptions').disabled, false);
  assert.equal(elements.get('enableBoot0').disabled, false, 'Ordinary scan permits opening the guide');
  assert.equal(vm.runInContext('targetResetConfirmed', context), false, 'Ordinary scan is not reset confirmation');
  assert.deepEqual(romOnly.commands.filter(command => command[0] === 'O'), [['O', 0x64]], 'Already-in-ROM detection automatically reads options');
  assert.equal(elements.get('flashBoot').hidden, false);
  assert.equal(elements.get('readOptions').hidden, true, 'Successful automatic reads need no manual read button');
  const unreadableOptions = new DiscoveryBridge([0x64]); unreadableOptions.optionReplies = ['ERR option read failed']; installDiscovery(unreadableOptions);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('bootRecovery', context), null);
  assert.equal(elements.get('enableBoot0').hidden, true);
  assert.equal(elements.get('flashBoot').hidden, true);
  assert.equal(vm.runInContext('optionStopped', context), true);
  assert.equal(unreadableOptions.writes, 0);
  assert.deepEqual(unreadableOptions.commands.filter(command => command[0] === 'O'), [['O', 0x64]]);
  const impostor = new DiscoveryBridge([0x64]); impostor.romValid = false; installDiscovery(impostor);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('romReady', context), false);
  assert.match(elements.get('deviceStatus').textContent, /no supported target/);
  assert.equal(elements.get('firmwareStep').hidden, true);
  assert.equal(elements.get('bootOptionsStep').hidden, true);

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
  // Exercise the dialog through the same event handlers used by its buttons.
  class WizardBridge extends DiscoveryBridge {
    constructor(addresses, profile = PROFILES.c011) { super(addresses, profile); this.optionReplies = [optionDump, summary, ack]; this.optionCommands = []; }
    async command(operation, values) {
      if (operation === 'O') { this.optionCommands.push(Array.from(values)); return this.optionReplies; }
      return super.command(operation, values);
    }
  }
  const wizard = new WizardBridge([0x49]); wizard.ids.set(0x49, 'DATA 90'); installDiscovery(wizard);
  elements.get('enableBoot0').onclick();
  assert.equal(elements.get('bootWizard').open, true);
  assert.match(elements.get('wizardText').textContent, /RST pin to GND/);
  assert.equal(wizard.commands.length, 0, 'Opening the guide does not talk to hardware');
  assert.equal(vm.runInContext('targetResetConfirmed', context), false);
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'select');
  assert.equal(elements.get('wizardDevice').value, String(0x49));
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'write');
  assert.equal(wizard.optionCommands.length, 0);
  await elements.get('wizardNext').onclick();
  assert.deepEqual(wizard.optionCommands, [[0x64, 0, 1, 1]]);
  assert.equal(vm.runInContext('wizardStage', context), 'verifyReset');
  assert.match(elements.get('wizardText').textContent, /3.3 V — never 5 V/);
  assert.match(elements.get('wizardText').textContent, /RST to GND/);
  // Escape is blocked while busy, and pending close requires confirmation.
  let cancelledEscape = false;
  vm.runInContext('busy = true;', context);
  elements.get('bootWizard').events.cancel({preventDefault() { cancelledEscape = true; }});
  assert.equal(cancelledEscape, true); assert.equal(elements.get('bootWizard').open, true);
  vm.runInContext('busy = false;', context);
  context.window.confirm = () => false;
  elements.get('wizardClose').onclick(); assert.equal(elements.get('bootWizard').open, true);
  context.window.confirm = () => true;
  elements.get('wizardClose').onclick(); assert.equal(elements.get('bootWizard').open, false);
  assert.equal(wizard.closed, undefined);
  elements.get('resumeOptions').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'verifyReset');
  // A bad BOOT0 strap must not be hidden by an automatic software boot key.
  wizard.addresses = [0x49]; wizard.romActive = false;
  const bootKeys = wizard.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length;
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'verifyReset');
  assert.equal(wizard.optionCommands.length, 1);
  assert.equal(wizard.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length, bootKeys);
  // Good physical reset into ROM; other application devices may remain on bus.
  wizard.addresses = [0x49, 0x64]; wizard.romActive = true;
  wizard.optionReplies = [optionDump, summary, verified];
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'done');
  assert.match(elements.get('wizardStatus').textContent, /^Verified:/);
  assert.deepEqual(wizard.optionCommands, [[0x64, 0, 1, 1], [0x64]]);
  await elements.get('wizardNext').onclick();
  assert.equal(elements.get('bootWizard').open, false);
  assert.equal(vm.runInContext('romReady', context), false);
  assert.equal(wizard.closed, undefined);

  // Normal flash boot keeps software ROM entry inside the same dialog.
  const normal = new WizardBridge([0x64]); installDiscovery(normal);
  elements.get('flashBoot').onclick();
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'write');
  assert.match(elements.get('wizardText').textContent, /not recommended/);
  await elements.get('wizardNext').onclick();
  assert.deepEqual(normal.optionCommands, [[0x64, 1, 1, 1]]);
  normal.addresses = [0x49]; normal.ids.set(0x49, 'DATA 90'); normal.romActive = false;
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'select');
  normal.optionReplies = [optionDump, summary, verified];
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'done');
  assert.deepEqual(normal.optionCommands, [[0x64, 1, 1, 1], [0x64]]);
  await elements.get('wizardNext').onclick();

  // Empty discovery stays in the dialog; multiple devices require a choice.
  const wizardSelection = new WizardBridge([]); installDiscovery(wizardSelection);
  elements.get('enableBoot0').onclick();
  await elements.get('wizardNext').onclick();
  assert.equal(elements.get('wizardNext').disabled, true);
  assert.match(elements.get('wizardStatus').textContent, /No I²C devices/);
  wizardSelection.addresses = [0x49, 0x4a];
  wizardSelection.ids.set(0x49, 'DATA 90'); wizardSelection.ids.set(0x4a, 'DATA 90');
  await elements.get('wizardScan').onclick();
  assert.equal(elements.get('wizardNext').disabled, true);
  elements.get('wizardDevice').value = String(0x4a);
  elements.get('wizardDevice').onchange();
  assert.equal(elements.get('wizardNext').disabled, false);
  await elements.get('wizardNext').onclick();
  assert.equal(wizardSelection.commands.filter(command => command[0] === 'W' && command[1] === 0x49).length, 0);
  assert.equal(wizardSelection.commands.filter(command => command[0] === 'W' && command[1] === 0x4a).length, 1);
  wizardSelection.optionReplies = [optionDump, summary, 'OK boot options already match; no write'];
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'done');
  assert.match(elements.get('wizardStatus').textContent, /no write was needed/);
  await elements.get('wizardNext').onclick();

  // Failed writes offer readback only; never retry the setter.
  const wizardFailure = new WizardBridge([0x64]); installDiscovery(wizardFailure);
  wizardFailure.optionReplies = ['ERR option write unconfirmed; do not retry'];
  elements.get('enableBoot0').onclick();
  await elements.get('wizardNext').onclick(); await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'error');
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'recoverReset');
  wizardFailure.optionReplies = [optionDump, summary]; // Lost snapshot is not verified.
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'error');
  assert.equal(vm.runInContext('optionPending', context), true);
  assert.deepEqual(wizardFailure.optionCommands, [[0x64, 0, 1, 1], [0x64]]);
  // C031 has its own ROM address, device ID, SRAM bound and GET frame.
  const c031Image = image.slice();
  new DataView(c031Image.buffer).setUint32(0, 0x20003000, true);
  assert.throws(() => validateImage(c031Image, PROFILES.c011), /stack/);
  assert.deepEqual(validateImage(c031Image, PROFILES.c031), c031Image);
  const blank31 = new DiscoveryBridge([0x63], PROFILES.c031); installDiscovery(blank31);
  elements.get('bootWizard').close();
  blank31.optionReplies = [optionDump, disabledSummary];
  vm.runInContext('target = null; firmware = {image: testImage};', context);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('target.name', context), 'C031');
  assert.equal(vm.runInContext('romReady', context), true);
  assert.equal(elements.get('bootRom').hidden, true, 'Hide for C031 ROM too');
  assert.equal(elements.get('firmwareStep').hidden, false);
  assert.equal(elements.get('bootOptionsStep').hidden, false);
  assert.equal(vm.runInContext('firmware', context), null, 'Detected family changes invalidate the selected image');
  assert.equal(blank31.writes, 0); assert.equal(blank31.erases.length, 0);
  assert.equal(blank31.goRequested, false);
  assert.deepEqual(blank31.commands.filter(command => command[0] === 'O'), [['O', 0x63]], 'C031 uses its own ROM option query');
  assert.equal(elements.get('enableBoot0').hidden, false);
  assert.equal(elements.get('flashBoot').hidden, true);
  const c031 = new ROM(PROFILES.c031), c031Rom = new Bootloader(c031, PROFILES.c031);
  await c031Rom.identify(); await c031Rom.erase(c031Image.length);
  const c031Padded = await c031Rom.program(c031Image);
  assert.deepEqual(await c031Rom.readMemory(base, c031Padded.length), c031Padded);
  assert.ok(c031.memory.slice(30720).every(v => v === 0x5A));
  await c031Rom.go(); assert.equal(c031.goRequested, true);
  const wrongId = new ROM({...PROFILES.c031, id: 0x443});
  await assert.rejects(() => new Bootloader(wrongId, PROFILES.c031).identify(), /Wrong STM32/);
  assert.equal(wrongId.writes, 0); assert.equal(wrongId.erases.length, 0);
  const smallFlash = new ROM(PROFILES.c031); smallFlash.flashKiB = 16;
  await assert.rejects(() => new Bootloader(smallFlash, PROFILES.c031).identify(), /32 KiB/);
  assert.equal(smallFlash.writes, 0); assert.equal(smallFlash.erases.length, 0);
  const running31 = new DiscoveryBridge([0x49], PROFILES.c031);
  running31.ids.set(0x49, 'DATA 91'); installDiscovery(running31);
  vm.runInContext('target = null;', context);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('target.name', context), 'C031');
  assert.equal(elements.get('bootRom').disabled, false);
  await vm.runInContext('bootIntoROM()', context);
  assert.equal(vm.runInContext('romReady', context), true);
  assert.ok(running31.commands.every(command => command[0] !== 'R' || ![0x63, 0x64].includes(command[1])));
  const wrongRom31 = new DiscoveryBridge([0x63], {...PROFILES.c031, id: 0x443}); installDiscovery(wrongRom31);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('romReady', context), false);
  assert.equal(wrongRom31.writes, 0);
  const smallRom31 = new DiscoveryBridge([0x63], PROFILES.c031); smallRom31.flashKiB = 16; installDiscovery(smallRom31);
  await vm.runInContext('scanDevices()', context);
  assert.equal(vm.runInContext('romReady', context), false);

  // Both ROM addresses are identified read-only; a mixed bus needs a choice.
  const mixed11 = new DiscoveryBridge([0x64]), mixed31 = new DiscoveryBridge([0x63], PROFILES.c031);
  const mixed = {command: async (operation, values) => operation === 'S' ? 'SCAN 63 64'
    : (values[0] === 0x63 ? mixed31 : mixed11).command(operation, values)};
  installDiscovery(mixed);
  await vm.runInContext('scanDevices()', context);
  assert.equal(elements.get('device').value, '');
  assert.match(elements.get('device').innerHTML, /C011 ROM/);
  assert.match(elements.get('device').innerHTML, /C031 ROM/);
  elements.get('device').value = String(0x64); await elements.get('device').onchange();
  assert.equal(vm.runInContext('target.name', context), 'C011');
  vm.runInContext('firmware = {image: testImage};', context);
  elements.get('device').value = String(0x63); await elements.get('device').onchange();
  assert.equal(vm.runInContext('target.name', context), 'C031');
  assert.equal(vm.runInContext('firmware', context), null);
  assert.deepEqual(mixed11.commands.filter(command => command[0] === 'O'), [['O', 0x64]]);
  assert.deepEqual(mixed31.commands.filter(command => command[0] === 'O'), [['O', 0x63]]);
  assert.equal(mixed11.writes + mixed31.writes, 0);
  assert.equal(mixed11.erases.length + mixed31.erases.length, 0);

  // Never switch families while an option write needs verification/recovery.
  installDiscovery(blank31);
  vm.runInContext('target = PROFILES.c011; optionPending = true;', context);
  await vm.runInContext('scanDevices()', context);
  elements.get('device').value = String(0x63); vm.runInContext('selectDevice()', context);
  assert.equal(vm.runInContext('target.name', context), 'C011');
  assert.equal(vm.runInContext('romReady', context), false);
  assert.equal(elements.get('readOptions').disabled, true);
  vm.runInContext('optionPending = false; optionStopped = true; selectDevice();', context);
  assert.equal(vm.runInContext('target.name', context), 'C011');
  assert.equal(vm.runInContext('romReady', context), false);
  const wizard31 = new WizardBridge([0x49], PROFILES.c031);
  wizard31.ids.set(0x49, 'DATA 91'); installDiscovery(wizard31);
  elements.get('enableBoot0').onclick();
  await elements.get('wizardNext').onclick(); await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'write');
  await elements.get('wizardNext').onclick();
  assert.deepEqual(wizard31.optionCommands, [[0x63, 0, 1, 1]]);
  wizard31.addresses = [0x63]; wizard31.romActive = true;
  wizard31.optionReplies = [optionDump, summary, verified];
  await elements.get('wizardNext').onclick();
  assert.equal(vm.runInContext('wizardStage', context), 'done');
  assert.deepEqual(wizard31.optionCommands, [[0x63, 0, 1, 1], [0x63]]);
  await elements.get('wizardNext').onclick();
  const write31 = new ROM(PROFILES.c031);
  context.testDevice = write31; context.testImage = c031Image;
  elements.get('verifyAfterFlash').checked = true;
  vm.runInContext('bridge = testDevice; romReady = true; firmware = {image: testImage};', context);
  await vm.runInContext('runOperation("write")', context);
  assert.equal(write31.goRequested, true);
  assert.ok(write31.options.every(v => v === 0xAA));
  assert.ok(write31.memory.slice(30720).every(v => v === 0x5A));
  assert.match(elements.get('status').textContent, /^Write verified/);
  process.stdout.write('PASS: file/firmware; boot-option framing and safety; discovery and ROM entry; guided reset/write/verify; physical BOOT0 check; dialog close/resume; read-only failure recovery.\n');
})().catch(error => { console.error(error); process.exitCode = 1; });

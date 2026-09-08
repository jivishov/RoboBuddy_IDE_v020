import assert from 'node:assert/strict';
import { LivePythonBridge } from '../../src/runtime/live-python-bridge.js';

class FakeSession {
  constructor(timestepSeconds) {
    this.timestepSeconds = timestepSeconds;
    this.calls = [];
  }
  async getDiagnostics() { this.calls.push(['getDiagnostics']); return { timestepSeconds: this.timestepSeconds }; }
  async advanceSteps(steps) { this.calls.push(['advanceSteps', steps]); return { simulationTimeSeconds: steps * this.timestepSeconds }; }
  async getObservation(options) { this.calls.push(['getObservation', options]); return { simulationTimeSeconds: 0, view: options.view }; }
  async sendCommand(command, options) { this.calls.push(['sendCommand', command, options]); return { status: 'accepted' }; }
  async cancelRun(reason) { this.calls.push(['cancelRun', reason]); return { cancelled: true }; }
}

const phase1 = new FakeSession(0.002);
const bridge1 = new LivePythonBridge(phase1);
await bridge1.advance(0.1, { controllerHz: 10 });
assert.deepEqual(phase1.calls.at(-1), ['advanceSteps', 50], 'controller frequency must not redefine the MuJoCo step count');

const so101 = new FakeSession(0.005);
const bridge2 = new LivePythonBridge(so101);
const advanced = await bridge2.advance(0.25, { controllerHz: 50 });
assert.deepEqual(so101.calls.at(-1), ['advanceSteps', 50]);
assert.equal(advanced.simulationTimeSeconds, 0.25);
await bridge2.advance(0);
assert.deepEqual(so101.calls.at(-1), ['getObservation', { view: 'ground_truth' }], 'zero seconds must not consume a physics step');
await assert.rejects(() => bridge2.advance(0.012), /integer number of 0.005s physics steps/);

const invalid = new LivePythonBridge(new FakeSession(null));
await assert.rejects(() => invalid.advance(0.1), /valid MuJoCo timestep/);

console.log('Live Python bridge physics-timestep checks: OK');

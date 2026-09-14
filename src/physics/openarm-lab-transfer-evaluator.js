export * from './openarm-lab-transfer-evaluator-base.js';
import { OpenArmLabTransferEvaluator as BaseEvaluator } from './openarm-lab-transfer-evaluator-base.js';

const PHASE_FLAGS = Object.freeze(['grasp','lift','transport','receivingRegion','support','release','settled','retreat','excessivePenetration','prohibitedContact']);

// Targeted correction layer: the base evaluator sets phase flags before its private event helper,
// which can suppress the corresponding event record. Preserve the base logic and synthesize only
// missing transitions after each observation; task credit still comes exclusively from the base
// read-only evaluator state.
export class OpenArmLabTransferEvaluator extends BaseEvaluator {
  observe(observation) {
    const before = this.snapshot()?.flags || {};
    const result = super.observe(observation);
    const after = result?.flags || {};
    const time = Number(observation?.simulationTimeSeconds);
    for (const flag of PHASE_FLAGS) {
      if (before[flag] === true || after[flag] !== true) continue;
      const alreadyRecorded = (this.events || []).some(event => event?.type === flag);
      if (!alreadyRecorded) this.events.push({ type: flag, simulation_time_s: Number.isFinite(time) ? time : null });
    }
    return this.snapshot();
  }
}

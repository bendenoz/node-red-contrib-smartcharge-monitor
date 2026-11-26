import { NodeDef } from "node-red";
import { IMMFilter } from "./imm-filter";

export type MonitorConfig = NodeDef & {
  cutoff?: number | string;
  peakCharge?: number | string;
  chargerEfficiency?: number | string;
  pwrStdev?: number | string;
};

export type Props = {
  imm: IMMFilter;
  /** last push timestamp */
  lastPush: number;
  /** Local plateau peak for current segment (W) */
  localPeak: number;
  /** Whether zK > 0 was entered during current segment */
  inDecay: boolean;
  /** Total Ws */
  energy: number;
  /** Peak detected power while not in decay */
  maxPwr: number;
  timeout: ReturnType<typeof setTimeout> | null;
  finishTimeout: ReturnType<typeof setTimeout> | null;
  finishing: boolean;
  offSent: boolean;
  /** Charge start time */
  startTime: number;
};

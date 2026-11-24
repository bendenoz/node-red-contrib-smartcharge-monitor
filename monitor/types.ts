import { IMMFilter } from "./imm-filter";

export type Props = {
  imm: IMMFilter;
  /** last push timestamp */
  lastPush: number;
  /** in h^1 ⋅ min, aka normalized time for a k of 1 h^-1 */
  triggerCusum: number;
  /** Total Ws */
  energy: number;
  /** Battery capacity, derived from last value */
  battCap: number;
  /** Max detected power when cusum == 0 */
  maxPwr: number;
  timeout: ReturnType<typeof setTimeout> | null;
  finishing: boolean;
  /** Charge start time */
  startTime: number;
};

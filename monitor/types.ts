import { KalmanFilter } from "./kalman-filter";

export type Props = {
  kfSlow: KalmanFilter;
  kfFast: KalmanFilter;
  /** last push timestamp */
  lastPush: number;
  /** in h^1 ⋅ min */
  cusum: number;
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

import { IIRFilter } from "./iir-filter";
import { KalmanFilter } from "./kalman-filter";
import { SimpleKalmanFilter } from "./simple-kalman-filter";
import { RollingDerivate } from "./sg-derivate";

export type Props = {
  slowFilter: KalmanFilter;
  fastFilter: KalmanFilter;
  /** display velocity */
  acceleration: IIRFilter;
  /** last timestamp */
  before: number;
  /** Normalized time, in minutes, for a k of 1 h^1 */
  cusum: number;
  /** Total Ws */
  energy: number;
  /** Battery capacity, derived from last value */
  battCap: number;
  /** Max detected power when cusum == 0 */
  maxPwr: number;
  timeout: ReturnType<typeof setTimeout> | null;
  decaying: boolean;
  finishing: boolean;
};

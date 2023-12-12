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
  cusum: number;
  /** Total Ws */
  energy: number;
  /** Battery capacity, derived from last value */
  battCap: number;
  timeout: ReturnType<typeof setTimeout> | null;
};

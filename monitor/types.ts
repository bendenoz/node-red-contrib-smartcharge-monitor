import { IIRFilter } from "./iir-filter";
import { KalmanFilter } from "./kalman-filter";
import { SimpleKalmanFilter } from "./simple-kalman-filter";
import { RollingDerivate } from "./sg-derivate";

export type Props = {
  value: KalmanFilter;
  accel: RollingDerivate;
  /** last timestamp */
  before: number;
  cusum: number;
  /** Total Ws */
  energy: number;
};

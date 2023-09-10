import { IIRFilter } from "./iir-filter";
import { KalmanFilter } from "./kalman-filter";
import { SimpleKalmanFilter } from "./simple-kalman-filter";
import { RollingDerivate } from "./sg-derivate";

export type Props = {
  slowFilter: KalmanFilter;
  fastFilter: KalmanFilter;
  accel: RollingDerivate;
  velocity: IIRFilter;
  /** last timestamp */
  before: number;
  cusum: number;
  /** Total Ws */
  energy: number;
};

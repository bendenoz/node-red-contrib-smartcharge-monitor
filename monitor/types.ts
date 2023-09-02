import { IIRFilter } from "./iir-filter";
import { KalmanFilter } from "./kalman-filter";
import { SimpleKalmanFilter } from "./simple-kalman-filter";

export type Props = {
  value: IIRFilter | KalmanFilter | SimpleKalmanFilter;
  longRate: IIRFilter;
  shortRate: IIRFilter;
  /** last timestamp */
  before: number;
  cusum: number;
  /** Total Ws */
  energy: number;
};

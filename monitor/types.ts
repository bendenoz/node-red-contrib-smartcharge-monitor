import { IIRFilter } from "./iir-filter";

export type Props = {
  value: IIRFilter;
  longRate: IIRFilter;
  shortRate: IIRFilter;
  /** last timestamp */
  before: number;
  cusum: number;
  /** Total Ws */
  energy: number;
};

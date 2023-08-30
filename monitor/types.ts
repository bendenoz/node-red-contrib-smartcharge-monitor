import { IIRFilter } from "./iir-filter";

export type Props = {
  pv?: number;
  count: number;
  value: IIRFilter;
  longRate: IIRFilter;
  shortRate: IIRFilter;
  lastSampleTime: number;
};

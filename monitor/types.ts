import { IIRFilter } from "./iir-filter";

export type Props = {
  value: IIRFilter;
  longRate: IIRFilter;
  shortRate: IIRFilter;
  lastSampleTime: number;
  cusum: number;
};

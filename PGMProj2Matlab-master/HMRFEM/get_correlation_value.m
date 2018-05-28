function [corrVal] = get_correlation_value(Xout, prob_map_shifted, bkgroundMsk)

XoutEdge = edge(Xout, 'canny');

XoutNobkgrndpxl = XoutEdge(bkgroundMsk>0);
XoutShifted = XoutEdge - mean(XoutNobkgrndpxl);
XoutShifted = XoutShifted.*(bkgroundMsk>0);

corrVal = sum(sum(prob_map_shifted.*XoutShifted));


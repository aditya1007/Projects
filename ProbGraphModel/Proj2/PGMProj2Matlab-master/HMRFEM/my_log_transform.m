function [opimg] = my_log_transform(ipimg)

opimg = log2(1+single(ipimg))/8;

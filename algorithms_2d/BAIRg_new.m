function [a,t,detail] = BAIRg_new(opts), if nargin<1,opts=struct;end, [a,t,detail]=bair_core2d('g',opts); end

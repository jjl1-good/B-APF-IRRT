function [a,t,detail] = BAIRy_new(opts), if nargin<1,opts=struct;end, [a,t,detail]=bair_core2d('y',opts); end

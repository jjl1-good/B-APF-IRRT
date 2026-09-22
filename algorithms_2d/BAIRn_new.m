function [a,t,detail] = BAIRn_new(opts), if nargin<1,opts=struct;end, [a,t,detail]=bair_core2d('n',opts); end

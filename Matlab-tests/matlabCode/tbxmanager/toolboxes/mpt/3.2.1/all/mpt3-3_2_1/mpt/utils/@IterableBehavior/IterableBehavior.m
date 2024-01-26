classdef IterableBehavior < handle
	% Basic class which provides a forEach() method to iterate over an
	% array
	%
	% In addition, we also provide the rejectArray() method, which throws a
	% nice error if the object is an array. Note that this method is
	% hidden (i.e., considered a private API), hence it will not show up in
	% Matlab's list of methods.
	
	methods
		function obj = IterableBehavior(obj)
		end
	end
	
	methods(Sealed=true)
		% sealed methods can not be redefined in a subclass
		function varargout = forEach(obj, fun, varargin)
			% Applies a given function to each element of an array.
			%
			% To apply method MYFUN to all elements of the array of
			% objects OBJ, you can use any of the following ways:
			%
			%   obj.forEach(@myfun)
			%   obj.forEach(@(x) myfun(x))
			%   obj.forEach(@(x) x.myfun)
			%
			% Note that forEach() will not return any outputs if not asked
			% to.
			%
			% Outputs can be requested as well:
			%
			%   [A, B, C] = obj.forEach(@myfun)
			%
			% If MYFUN generates all its outputs as scalars, then A, B, and
			% C will be arrays with as many elements as there are elements
			% in the array OBJ.
			%
			% If outputs generated by MYFUN have different sizes for
			% different elements of OBJ, then you must set the
			% "UniformOutput" option to false:
			%
			%    [A, B, C] = obj.forEach(@myfun, 'UniformOutput', false)
			%
			% In this case A, B, and C will be cell arrays, with A{i},
			% B{i} and C{i} containing the output of MYFUN applied to the
			% i-th element of OBJ.
			%
			% In short, the forEach() method works exactly as arrayfun()
			% and cellfun(), hence see "help arrayfun" for a more throught
			% description.
			
			if ~isempty(varargin)
				% parse input arguments
				ip = inputParser;
				ip.addParamValue('UniformOutput', true, @islogical);
				ip.parse(varargin{:});
				Options = ip.Results;
			else
				Options.UniformOutput = true;
			end
			
			if nargout==0
				% no outputs, just execute the method
				for i = 1:numel(obj)
					fun(obj(i));
				end
				
			else
				% preallocate outputs
				for i = 1:nargout
					if Options.UniformOutput
						% we will concatenate the i-th output to an array
						varargout{i} = [];
					else
						varargout{i} = cell(1, numel(obj));
					end
				end
				
				for i = 1:numel(obj)
					out = cell(1, nargout);
					[out{:}] = fun(obj(i));
					if Options.UniformOutput
						% check whether all outputs are scalar
						for j = 1:nargout
							if ~isscalar(out{j})
								error('Non-scalar in Uniform output, at index %d, output %d.\nSet ''UniformOutput'' to false.', i, j);
							end
						end
						
						% put the elements to an array
						try
							for j = 1:nargout
								if numel(varargout{j})==0
									varargout{j} = out{j};
								else
									varargout{j} = [varargout{j} out{j}];
								end
							end
						catch
							% concatenation failed, most probably due to a
							% mixup of types
							error('Mismatch in type of outputs, at index %d, output %d.\nSet ''UniformOutput'' to false.', i, j);
						end
					else
						% non-uniform outputs are simply returned as cells
						for j = 1:nargout
							varargout{j}{i} = out{j};
						end
					end
				end
			end
			
		end
	end
	
	methods(Hidden, Sealed=true)
		% private APIs
		
		function str = rejectArray(obj)
			% Throw an error if object is an array.
			%
			% This helper is to be used in all methods which do not support
			% arrays:
			%
			%    function mymethod(obj)
			%
			%       error(obj.rejectArray());
			%       ...
			%    end
			
			if numel(obj)>1
				str = 'This method does not support arrays. Use the forEach() method.';
			else
				str = '';
			end
		end
	end
end

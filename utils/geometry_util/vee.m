function a_vee = vee(A)
if size(A,1) ~= 3 && size(A,2) ~= 3  
    error('Matrix is not 3x3. Cannot compute vee');
end

if norm(A+A')~=0
    error('Matrix is not skew-symmetric. Cannot compute vee');   
end

a_vee = [-A(2,3) A(1,3) -A(1,2)]';

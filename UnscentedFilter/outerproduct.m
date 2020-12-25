 function Mat = outerproduct(A,B)

  if size(A) ~= size(B)
      warning("To do an inner proudct, they must be the proper size")
  else 
      Mat = A(:)*B(:).';
  end 


 end 

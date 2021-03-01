
q0 = 0.707;
q1 = 0;
q2 = 0.707;
q3 = 0;


C11 = (q0^2)+(q1^2)-(q2^2)-(q3^2);
C12 = (2*(q1*q2+q0*q3));
C13 = (2*(q1*q3-q0*q2));
%C21 = 2*(q1*q2-q0*q3)
%C22 = (q0^2)-(q1^2)+(q2^2)-(q3^2)
C23 = (2*(q2*q3+q0*q1));
%C31 = 2*(q1*q3+q0*q2)
%C32 = 2*(q2*q3-q0*q1)
C33 = ((q0^2)-(q1^2)-(q2^2)+(q3^2));

%C = [C11 C12 C13; C21 C22 C23; C31 C32 C33]

%DCM = [ctheta*calpha ctheta*salpha -stheta; sphi*stheta*calpha-cphi*salpha
%sphi*stheta*salpha+cphi*calpha sphi*ctheta; cphi*stheta*calpha+sphi*salpha
%cphi*stheta*salpha-sphi*calpha cphi*calpha]

theta = asind(-C13);


if C12 >= 0 && C11 >= 0
    
    if C23 >= 0 && C33 >= 0

        alpha = atand(C12/C11);
        phi = atand(C23/C33);

    elseif C23 >= 0 && C33 <= 0 

        alpha = atand(C12/C11);
        phi = atand(C23/C33) + 180;
        
    elseif C23 <= 0 && C33 <= 0

         alpha = atand(C12/C11);
         phi = atand(C23/C33) - 180;

    elseif C23 <= 0 && C33 >= 0

        alpha = atand(C12/C11);
        phi = atand(C23/C33);
        
         
    else
        
        alpha = 0;
        phi = 0;


    end

    
elseif C12 >= 0 && C11 <= 0 
    
    if C23 >= 0 && C33 >= 0

        alpha = atand(C12/C11) + 180;
        phi = atand(C23/C33);

    elseif C23 >= 0 && C33 <= 0 

        alpha = atand(C12/C11) + 180;
        phi = atand(C23/C33) + 180;
        
    elseif C23 <= 0 && C33 <= 0

         alpha = atand(C12/C11) + 180;
         phi = atand(C23/C33) - 180;

    elseif C23 <= 0 && C33 >= 0

        alpha = atand(C12/C11) + 180;
        phi = atand(C23/C33);
        
         
    else
        
        alpha = 0;
        phi = 0;


    end
    
    
elseif C12 <= 0 && C11 <= 0
    
    if C23 >= 0 && C33 >= 0

        alpha = atand(C12/C11) - 180;
        phi = atand(C23/C33);

    elseif C23 >= 0 && C33 <= 0 

        alpha = atand(C12/C11) - 180;
        phi = atand(C23/C33) + 180;
        
    elseif C23 <= 0 && C33 <= 0

         alpha = atand(C12/C11) - 180;
         phi = atand(C23/C33) - 180;

    elseif C23 <= 0 && C33 >= 0

        alpha = atand(C12/C11) - 180;
        phi = atand(C23/C33);
        
         
    else
        
        alpha = 0;
        phi = 0;


    end
     
     
elseif C12 <= 0 && C11 >= 0
    
    if C23 >= 0 && C33 >= 0

        alpha = atand(C12/C11);
        phi = atand(C23/C33);

    elseif C23 >= 0 && C33 <= 0 

        alpha = atand(C12/C11);
        phi = atand(C23/C33) + 180;
        
    elseif C23 <= 0 && C33 <= 0

         alpha = atand(C12/C11);
         phi = atand(C23/C33) - 180;

    elseif C23 <= 0 && C33 >= 0

        alpha = atand(C12/C11);
        phi = atand(C23/C33);
        
    else
        
        alpha = 0;
        phi = 0;

    end
    
else
    
    alpha = 0;
    phi = 0;
    
end


function x=giaiPTBac2(a,b,c) 
    % kiem tra cac he so
    if a==0
        if b==0         x=9999;     %printf("Phuong trinh vo nghiem!");
        else            x=-c/b;
        end
    else
        delta = b*b - 4*a*c;
        % tinh nghiem
        if delta>0
            x1 = ((-b + sqrt(delta)) / (2*a));
            x2 = ((-b - sqrt(delta)) / (2*a));
            if (x1<0 && x2<0)   x=9999;
            elseif (x1<=0)      x=x2;
            elseif (x2<=0)      x=x1;
            else
                if (x1>x2)      x=x2;
                else            x=x1;
                end
            end   
        elseif delta==0      x = (-b / (2 * a));
        else                        x=9999;     %printf("Phuong trinh vo nghiem!");            
        end
    end
end
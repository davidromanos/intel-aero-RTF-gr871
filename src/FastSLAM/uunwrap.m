function [ out,oldin ] = uunwrap( in,state,oldinput )
if(abs(in-oldinput)>pi*1.5)    
    if(in-oldinput >0)
        state = state + (in-oldinput) - 2*pi;
    else
        state = state + (in-oldinput) + 2*pi;
    end      
else
    state = state + in-oldinput;
end
oldin = in;
out = state;
end


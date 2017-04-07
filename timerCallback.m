function timerCallback(~, ~, cl_cycle_primi, cl_cycle_mc, cl_cycle_llbeh)
    
    global  uptillnow_p;
    global  uptillnow_pc;
    global  uptillnow_cm;
    global  uptillnow_cmc;
    global  uptillnow_llb;
    global  uptillnow_llbc;
    
    uptillnow_p             =   true;
    for i=1:cl_cycle_primi
        uptillnow_pc(i)     =   true;
    end
    
    uptillnow_cm            =   true;
    for i=1:cl_cycle_mc
        uptillnow_cmc(i)    =   true;
    end
    
    uptillnow_llb           =   true;
    for i=1:cl_cycle_llbeh
        uptillnow_llbc(i)   =   true;
    end
    
end
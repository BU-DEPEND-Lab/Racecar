def = legacy_code("initialize");
def.SFunctionName = "new_1";

def.OutputFcnSpec = 'double y1 = add_2(double u1, double u2)';

def.SourceFiles = {'main.c'};

def.LibPaths = {}

def.HeaderFiles = {'obj_det.h'};

legacy_code('sfcn_cmex_generate', def);

legacy_code('compile', def);
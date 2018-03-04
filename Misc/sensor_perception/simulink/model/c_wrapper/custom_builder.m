def = legacy_code("initialize");
def.SFunctionName = "new_1";

def.SourceFiles = {'obj_detection_custom.cpp'};


arch = computer('arch');


ocvcgDir = fullfile(matlabroot,'toolbox','vision','builtins','src','ocvcg');
ocvconfig.include   = fullfile(ocvcgDir,'opencv','include');
cvstocvutil.include = fullfile(matlabroot, 'extern','include');
ocvconfig.sharedLibraries = fullfile(matlabroot, 'bin', arch);
includes = {['-I' ocvconfig.include],['-I' cvstocvutil.include]};


def.IncPaths = {ocvconfig.include};
def.LibPaths = {ocvconfig.sharedLibraries};
def.HostLibFiles = {'libopencv_core.so.3.1.0', 'libopencv_imgproc.so.3.1.0'...
            'libopencv_objdetect.so.3.1.0', 'libopencv_highgui.so.3.1.0',...
            'libopencv_imgcodecs.so.3.1.0'};
def.OutputFcnSpec = 'double y1 = obj_det(double u1, double u2)';
def.HeaderFiles = {'obj_det.h'};
        


legacy_code('sfcn_cmex_generate', def);
legacy_code('compile', def, '-g');
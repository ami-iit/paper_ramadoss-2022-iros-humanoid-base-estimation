function install_idyntree_bindings(varargin)
% authored by Silvio Traversaro
% modified by Prashanth Ramadoss
% install iDynTree matlab bindings in the local directory
% setup a default install path
% or allow the user to specify path as
% install_iDynTree('installPrefix', '<path-to-install>')
p = inputParser;
default_install_prefix = fullfile(pwd, '../deps/', 'idyntree-matlab');
p.addOptional('installPrefix', default_install_prefix);
p.parse(varargin{:});

install_prefix = p.Results.installPrefix;

% If not on Windows, check if the install path contains a space as this
% creates an installation error later (see
% https://github.com/robotology/robotology-superbuild/issues/780)
if ~ispc
    if contains(strip(install_prefix), ' ')
        fprintf('Install directory path %s contains a space.\n', install_prefix)
        fprintf('Please use install_idyntree_bindings function in a directory path that does not have spaces.\n');
        return;
    end
end

setup_script = fullfile(pwd, 'idyntree_bindings_setup.m');

if exist(install_prefix)
    fprintf('Directory %s already present.\n', install_prefix);
    fprintf('Please use the existing installation or delete the existing installation to proceed with the install.\n');
    return;
end

fprintf('Installing iDynTree Matlab bindings in %s\n', install_prefix);

% The install url is created following
mambaforge_url_prefix = 'https://github.com/conda-forge/miniforge/releases/latest/download/';
if ispc
    mambaforge_installer_name = 'Mambaforge-Windows-x86_64.exe';
elseif ismac
    [~, uname_m] = system('uname -m');
    % Remove newline
    uname_m = strip(uname_m);
    mambaforge_installer_name = sprintf('Mambaforge-MacOSX-%s.sh', uname_m);
elseif isunix
    [~, uname] = system('uname');
    % Remove newline
    uname = strip(uname);
    [~, uname_m] = system('uname -m');
    % Remove newline
    uname_m = strip(uname_m);
    mambaforge_installer_name = sprintf('Mambaforge-%s-%s.sh', uname, uname_m);
end

fprintf('Downloading mambaforge installer \n');
mambaforge_installer_url = strcat(mambaforge_url_prefix, mambaforge_installer_name);
websave(mambaforge_installer_name, mambaforge_installer_url);
fprintf('Download of mambaforge installer completed\n');

% See https://github.com/conda-forge/miniforge#non-interactive-install
fprintf('Installing mambaforge\n');
if ispc
    system(sprintf('start /wait "" %s /InstallationType=JustMe /RegisterPython=0 /S /D=%s', mambaforge_installer_name, install_prefix));
    conda_full_path = fullfile(install_prefix, 'condabin', 'conda.bat');
    % On Windows, the files in conda are installed in the Library
    % subdirectory of the prefix
    idyntree_install_prefix = fullfile(install_prefix, 'Library');
elseif isunix
    system(sprintf('sh %s -b -p "%s"', mambaforge_installer_name, install_prefix));
    conda_full_path = fullfile(install_prefix, 'bin', 'conda');
    idyntree_install_prefix = install_prefix;
end
fprintf('Installation of mambaforge completed\n');

if ~exist(install_prefix, 'dir')
    fprintf('Installation in %s failed for unknown reason.\n', install_prefix);
    fprintf('Please open an issue at https://github.com/ami-iit/paper_ramadoss-2022-ral-humanoid-base-estimation/issues/new .\n');
    return;
end

% Install all the idyntree-matlab-bindings
fprintf('Installing idyntree-matlab-bindings from robotology\n');
system(sprintf('"%s" install -y -c conda-forge -c robotology idyntree-matlab-bindings', conda_full_path));
fprintf('Installation of idyntree-matlab-bindings completed\n');

fprintf('Creating setup script in %s\n', setup_script);
% Generate idyntree_bindings_setup.m
setupID = fopen(setup_script,'w');
fprintf(setupID, '%% Specify OS-specific locations\n');
fprintf(setupID, 'if ispc\n');
fprintf(setupID, '    rob_env_sep = ";";\n');
fprintf(setupID, '    rob_shlib_install_dir = "bin";\n');
fprintf(setupID, 'else\n');
fprintf(setupID, '    rob_env_sep = ":";\n');
fprintf(setupID, '    rob_shlib_install_dir = "lib";\n');
fprintf(setupID, 'end\n');
fprintf(setupID, '\n');
fprintf(setupID, '%% Install prefix (hardcoded at generation time)\n');
fprintf(setupID, 'idyntree_install_prefix = "%s";\n', idyntree_install_prefix);
fprintf(setupID, '\n');
fprintf(setupID, '%% Add directory to MATLAB path\n');
fprintf(setupID, 'addpath(fullfile(idyntree_install_prefix,"mex"));\n');
fprintf(setupID, '\n');
fprintf(setupID, '%% Append required values to system environment variables\n');
fprintf(setupID, 'setenv("PATH",strcat(fullfile(idyntree_install_prefix,"bin"), rob_env_sep, getenv("PATH")));\n');
fclose(setupID);

fprintf('Deleting mambaforge installer\n');
delete(mambaforge_installer_name);

fprintf('idyntree-matlab-bindings is successfully installed!\n');
fprintf('Please run %s before using the packages,\n',setup_script)
fprintf('or just add that script to your startup.m file, to run it whenever you open MATLAB.\n');
fprintf('To uninstall idyntree-matlab-bindings, just delete the folder %s .\n', install_prefix);


end


# Sourced by the run scripts (after they cd to the repo root): where the
# executables, the reference shelf and the user's media live on THIS host, so
# the same scripts run on Windows (Git Bash, MSVC build) and on Linux (GCC).
#
#   OPM_G4RUN    the g4run to run          (the MSVC Release .exe, else the Ninja/Make build)
#   OPM_SCRATCH  the shelf: roms/, ati/, hd.img, snapshots, logs
#                (../scratch/openpowermac when it exists, else scratch/ in the repo)
#   OPM_MEDIA    the folder holding the install CDs and game images
#
# Anything host-specific that should not be committed (a media folder under a
# home directory, say) goes in scratch/local.env, which is read first.
[ -f scratch/local.env ] && . scratch/local.env
if [ -z "${OPM_G4RUN:-}" ]; then
  if [ -x ./build/tools/g4run/Release/g4run.exe ]; then OPM_G4RUN=./build/tools/g4run/Release/g4run.exe
  else OPM_G4RUN=./build/tools/g4run/g4run; fi
fi
if [ -z "${OPM_SCRATCH:-}" ]; then
  if [ -d ../scratch/openpowermac ]; then OPM_SCRATCH=../scratch/openpowermac; else OPM_SCRATCH=scratch; fi
fi
OPM_MEDIA="${OPM_MEDIA:-/c/Users/gamer/Downloads/mac stuff/Open Power Mac}"

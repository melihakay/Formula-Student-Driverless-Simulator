# Run headless or not
SDL_VIDEODRIVER_VALUE=''
if [ "$MFR_FSDS_MAP" = "headless" ]; then
  SDL_VIDEODRIVER_VALUE="offscreen"
fi
bash ~/Formula-Student-Driverless-Engine/FSDS.sh $MFR_FSDS_MAP -windowed -ResX=1280 -ResY=720

rm x64\Release\*d.dll
rm x64\Release\*.pdb
rm x64\Release\*.lib
rm x64\Release\*.exp
rm x64\Release\isolines.dll
rm x64\Release\EzLasLib.dll
rm x64\Release\GF.UI.Components.Mfc.Private.dll
rm x64\Release\GF.Spatial.Shape.dll
rm x64\Release\GF.Spatial.Lidar.dll
rm x64\Release\GF.Spatial.Maps.dll
rm x64\Release\GF.Spatial.Raster.dll
rm x64\Release\GF.Core.dll
rm x64\Release\concrt140.dll

$compress = @{
  Path = "x64\Release\*"
  CompressionLevel = "Fastest"
  DestinationPath = "Symmetry_Deploy.Zip"
}

Compress-Archive @compress

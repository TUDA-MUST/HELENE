echo "Hiermit werden die Verküpfungen zum Starten von Helene hinzugefügt"
mkdir -p ~/.local/share/icons
mkdir -p ~/.local/share/applications
cp HeleneIcon.png ~/.local/share/icons/HeleneIcon.png
cp HeleneIconSim.png ~/.local/share/icons/HeleneIconSim.png
cp Icon_Log_Everything.png ~/.local/share/icons/Icon_Log_Everything.png
cp Icon_Log_Norm.png ~/.local/share/icons/Icon_Log_Norm.png

cp Start_helene.desktop ~/.local/share/applications/Start_helene.desktop
cp Start_helene_simulation.desktop ~/.local/share/applications/Start_helene_simulation.desktop
cp Start_FULL_Datalogger.desktop ~/.local/share/applications/Start_FULL_Datalogger.desktop
cp Start_Small_DataLogger.desktop ~/.local/share/applications/Start_Small_DataLogger.desktop
echo "Fertig :)"
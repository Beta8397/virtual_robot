package system.gui.menus.configmenu;

import system.gui.HALMenu;
import system.gui.Payload;
import system.gui.SelectionZone;
import system.gui.event.DataPacket;
import system.gui.viewelement.TextElement;
import system.gui.viewelement.eventlistener.EntireViewButton;
import util.exceptions.ExceptionChecker;
import util.exceptions.HALConfigException;

/**
 * The screen that gets displayed when the config system is automatically running a specific config file
 * <p>
 * Creation Date: 9/14/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see ConfigStartingMenu
 * @see ConfigConstants
 * @see HALMenu
 * @see system.gui.HALGUI
 * @see system.gui.viewelement.eventlistener.ViewButton
 * @see EntireViewButton
 * @see Payload
 * @see SelectionZone
 * @see system.robot.Robot
 * @since 1.1.0
 */
public class AutorunScreen extends HALMenu {

    /**
     * The AutorunScreen's init function.
     *
     * @param payload The payload passed to this menu.
     * @throws HALConfigException Throws this exception when the autorun screen is inflated without an autorun config file being specified.
     *
     * @see ConfigStartingMenu
     * @see ConfigConstants
     * @see HALMenu
     * @see system.gui.HALGUI
     * @see system.gui.viewelement.eventlistener.ViewButton
     * @see EntireViewButton
     * @see Payload
     * @see SelectionZone
     * @see system.robot.Robot
     */
    @Override
    protected void init(Payload payload) {
        selectionZone = new SelectionZone(0, 0);

        ExceptionChecker.assertTrue(payload.idPresent(ConfigConstants.CONFIG_FILE_NAME_ID), new HALConfigException("Could not find autorun config file."));
        String configFilename = payload.get(ConfigConstants.CONFIG_FILE_NAME_ID);

        addItem(new TextElement("Autorunning " + configFilename + ", press the select button to select a different file to run."));
        addItem(new EntireViewButton()
                .onClick(payload.get(ConfigConstants.SELECT_BUTTON_ID), (DataPacket packet) -> {
                    payload.remove(ConfigConstants.CONFIG_FILE_NAME_ID);

                    //runs in standalone mode because you are selecting a file, but also saves selected file to autorun.
                    payload.add(ConfigConstants.STANDALONE_MODE_ID, true);
                    payload.add(ConfigConstants.SAVE_TO_AUTORUN_ID, true);
                    gui.inflate(new ConfigStartingMenu(payload));
                })
                .addBackgroundTask((DataPacket packet) -> {
                    if (gui.getRobot().isStarted()) gui.removeCurrentStack();
                }));

    }
}
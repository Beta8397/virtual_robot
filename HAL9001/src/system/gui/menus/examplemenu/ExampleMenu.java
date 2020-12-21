package system.gui.menus.examplemenu;

import system.gui.DynamicSelectionZone;
import system.gui.HALMenu;
import system.gui.Payload;
import system.gui.event.DataPacket;
import system.gui.menus.TextSelectionMenu;
import system.gui.viewelement.TextElement;
import system.gui.viewelement.eventlistener.EntireViewButton;
import system.gui.viewelement.eventlistener.ViewButton;
import util.control.Button;

@DynamicSelectionZone(pattern = {true})
public class ExampleMenu extends HALMenu {
    @Override
    protected void init(Payload payload) {
        if (payload.idPresent(TextSelectionMenu.ENTERED_TEXT_ID)) {
            System.out.println("Entered text: " + payload.get(TextSelectionMenu.ENTERED_TEXT_ID));
        }

        //selectionZone = new SelectionZone(1,2);
        addItem(new ViewButton("# | Fun Times")
                .onClick(new Button<>(1, Button.BooleanInputs.x), (DataPacket packet) -> {
                    gui.inflate(new ExampleMenu2());
                })
        );
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new ViewButton("# | More fun stuff")
                .onClick(new Button<>(1, Button.BooleanInputs.a), (DataPacket packet) -> {
                    gui.inflate(new ExampleMenu2());
                }));
        addItem(new EntireViewButton()
                .onClick(new Button<>(1, Button.BooleanInputs.b), (DataPacket packet) -> {
                    gui.inflate(new ExampleMenu3());
                }));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new EntireViewButton()
                 .onClick(new Button<>(1, Button.BooleanInputs.y), (DataPacket packet) -> {
                    gui.forward();
                 }));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new TextElement("# | LOOK! its new text!"));
        addItem(new TextElement("# | You found the end of the page!"));
    }
}

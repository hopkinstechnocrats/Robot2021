package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import java.util.ArrayList;

public class PixySubsystem extends SubsystemBase {

    private final Pixy2 pixy;

    public PixySubsystem() {
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
        pixy.setLamp((byte) 1, (byte) 1);
        pixy.setLED(200, 30, 255);
    }

    public void periodic() {
        pixy.setLamp((byte) 1, (byte) 1);
    }

    public Pixy2CCC.Block getBiggestBlock() {
        // Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
        // does not wait for new data if none is available,
        // and limits the number of returned blocks to 25, for a slight increase in efficiency
        int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
        if (blockCount <= 0) {
            return null; // If blocks were not found, stop processing
        }
        ArrayList<Pixy2CCC.Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
        Pixy2CCC.Block largestBlock = null;
        for (Pixy2CCC.Block block : blocks) { // Loops through all blocks and finds the widest one
            if (largestBlock == null) {
                largestBlock = block;
            } else if (block.getWidth() > largestBlock.getWidth()) {
                largestBlock = block;
            }
        }
        return largestBlock;
    }

}

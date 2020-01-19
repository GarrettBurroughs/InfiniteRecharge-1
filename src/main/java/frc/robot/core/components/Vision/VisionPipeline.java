/** 
 * Vision Pipeline is responsible for using algorithms to get data from the vision feed
 */

package edu.wpi.first.wpilibj.vision;
import org.opencv.core.Mat;

public interface VisionPipeline {
    /**
     * Takes in image input and outputs the result, which is then fed into another 
     * algorithm, which feeds into another and so on until we get the desired result.
     */

    void process(Mat image); 
}



import { useEffect, useRef, useState } from "react";
import { core } from "../store/core";


const Camera = (props) => {
    const imgRef = useRef(null)
    const {cameraData, camera_qr_data} = core()
    const [cam, setcam] = useState("front") 

    useEffect(function(){
        if(cam === "front"){
            if (cameraData) {
                const imageUrl = `data:image/png;base64,${cameraData}`
                imgRef.current.src = imageUrl;

                // Clean up the object URL when component is unmounted
                return () => {
                    URL.revokeObjectURL(imageUrl);
                };
            }
        }
        else if (cam === "back"){
            if (camera_qr_data) {
                const imageUrl = `data:image/png;base64,${camera_qr_data}`
                imgRef.current.src = imageUrl;

                // Clean up the object URL when component is unmounted
                return () => {
                    URL.revokeObjectURL(imageUrl);
                };
            }
        }
 
    },[cameraData, camera_qr_data])

    const onCameraChange = function(e){
        setcam(e.target.value)
    }

    return (
        <>
            <div className="cam-state ">
                <select name="camera" id="camera" className="ml-2 h-10 bg-transparent text-white border" onChange={onCameraChange}>
                    <option value="front" className="text-black">Main camera</option>
                    <option value="back"  className="text-black">QR code camera</option>
                </select>
            </div>
            <div className="w-full h-full ">
                <img  alt="turtlebot camera output" className="w-full h-full object-cover  " ref={imgRef}/>
            </div>
        </>
    );
}

export default Camera;
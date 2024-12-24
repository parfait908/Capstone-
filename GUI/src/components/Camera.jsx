import { useEffect, useRef, useState } from "react";
import { clientStore } from "../store/clientStore";


const Camera = (props) => {
    const imgRef = useRef(null)
    const {cameraData, camera_qr_data} = clientStore()
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
            <div className="cam-state">
                <span className="text-white">Camera state </span>
                <label className="switch">
                    <input type="checkbox" name="switch" id="switch" />
                    <span className="slider round"></span>
                </label>
                <select name="camera" id="camera" className="ml-2 h-full bg-transparent text-white border" onChange={onCameraChange}>
                    <option value="front">Main camera</option>
                    <option value="back">QR code camera</option>
                </select>
            </div>
            <div className="w-full h-full ">
                <img  alt="turtlebot camera output" className="w-full h-full object-cover  " ref={imgRef}/>
            </div>
        </>
    );
}

export default Camera;
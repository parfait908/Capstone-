
import RemoteButton from "./Remotebutton"
import Map from "./Map"
import Camera from "./Camera";
import { clientStore } from "../store/clientStore";
import Speed from "./Speed";

const Remote = (props) => {

    const {sendDirection} = clientStore()
    const onReset = function(e){
        sendDirection("STOP")
    }
    return (
        <div className="w-full h-full overflow-hidden flex flex-row bg-slate-800">
            <section className="w-1/2 p-2">
                <div className="w-full h-2/3 flex flex-col items-center  border rounded-lg overflow-hidden">
                    <Camera/>
                </div>
                <div className="w-full h-[calc((100%/3)-8px)] border rounded-lg mt-2 flex flex-row">
                    <div className="circle-section flex items-center justify-center  w-1/2 h-full">
                        <div className="circle-container relative ">
                            <button className="center-circle" onClick={onReset}>r</button>
                            <RemoteButton direction="UP"/>
                            <RemoteButton direction="DOWN"/>
                            <RemoteButton direction="LEFT"/>
                            <RemoteButton direction="RIGHT"/>
                        </div>
                    </div>
                    <div className=" w-1/2 h-full">
                        <Speed/>
                    </div>

                </div>
            </section>
            <section className="w-1/2 flex items-center justify-center p-2">
                <div  className="w-full h-full border rounded-lg flex flex-col items-center relative overflow-hidden">
                    <Map ></Map>

                </div>

            </section>
    </div>
    );
}

export default Remote;
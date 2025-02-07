import { core } from "../store/core";

const Speed = (props) => {
    const {speed} = core()
    return (
        <div className="flex items-center justify-center w-full h-full flex-col">
            <p className="text-4xl text-white">{speed.toFixed(2)} m/s</p>
        </div>
    );
}

export default Speed;
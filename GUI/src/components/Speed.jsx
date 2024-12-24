import { clientStore } from "../store/clientStore";

const Speed = (props) => {
    const {speed} = clientStore()
    return (
        <div className="flex items-center justify-center w-full h-full flex-col">
            <p className="text-4xl text-white">{speed.toFixed(2)} m/s</p>
        </div>
    );
}

export default Speed;
import React, { useEffect, useRef, useState, useCallback } from 'react';
import SimpleTask from './SimpleTask';
import { core } from '../store/core';
import toast from 'react-hot-toast';

const Task = (props) => {
    const { sendTask, mapping, zone, station , processState} = core();
    const [zones, setZones] = useState([]);
    const [selected, setSelected] = useState("");
    const [task, setTask] = useState("");
    const [params, setParams] = useState([]);
    const [lcs, setLcs] = useState(false);
    const ref = useRef();

    const selection = useCallback((name) => {
        setSelected(name);
    }, []);

    useEffect(() => {
        if (zone != null) {
            setZones(zone);
            setLcs(zone.length > 0);
        }
    }, [zone]);

    useEffect(() => {
        setParams([]);
        setTask("");  
    }, [processState]);
    const set = () => {
        if (params.length === 6) return toast.error("You can't load and unload more than 6 times");

        if (selected === "") return toast.error("Select a zone");

        let type = ref.current.value === "Loading" ? "L" : "U";
        if (type === "U") {
            let param = params[params.length - 1];
            if (param) {
                let key = Object.keys(param)[0];
                if (param[key] !== "Loading") {
                    return toast.error("You can't unload without loading");
                } else if (key === selected) {
                    return toast.error("You can't unload from a zone you've loaded from");
                }
            } else {
                return toast.error("You need to load first");
            }
        }else if( type === "L"){
            let param = params[params.length - 1];
            if (param) {
                let key = Object.keys(param)[0];
                if (param[key] === "Loading") {
                    return toast.error("You can't load twice");
                }
            }
        }

        setTask(`${task}${selected}(${type}) -> `);
        setParams([...params, { [selected]: ref.current.value }]);
        setSelected("");
    };

    const send = () => {
        if (params.length === 0) return toast.error("No task to send");
        sendTask(params)
        setParams([]);
        setTask("");
    };

    const tasks = zones.map((element) => (
        <SimpleTask 
            name={element.location} 
            selection={selection} 
            selected={selected === element.location} 
            key={element.location}
        />
    ));

    return (
        <div className="w-full h-full px-5 pt-5 relative">
            <button className='w-full h-12 bg-slate-400 text-white' onClick={mapping}>Mapping</button>
            <input className='w-full h-12 bg-white mt text-black px-5 outline-none mt-8' 
                   type="text" value={task} name="task" id="task" placeholder='Task display' disabled />
            
            {lcs && (
                <div className='mt-4 w-full'>
                    <p>Select a zone and choose a task type</p>
                    <div className='grid grid-cols-4 gap-4 mt-4'>
                        {tasks}
                    </div>
                    <select name="taskType" className='mt-4 w-full h-12 px-5 outline-none' id="taskType" ref={ref}>
                        <option value="Loading">Loading</option>
                        <option value="Unloading">Unloading</option>
                    </select>
                    <button className='w-full h-12 bg-slate-400 text-white mt-4' onClick={set}>
                        Set
                    </button>
                </div>
            )}
            {
                processState == "Processing" && (
                    <div className='w-full h-12 bg-red-500 text-white mt-4 flex items-center justify-center'>
                        <p className='text-black'>current task : {task}</p>
                    </div>
            )}
            <div className='w-full h-12 absolute bottom-2 left-0 px-5'>
                <button className='w-full h-full bg-slate-400 text-white' onClick={send}>
                    Send task
                </button>
            </div>
        </div>
    );
};

export default Task;

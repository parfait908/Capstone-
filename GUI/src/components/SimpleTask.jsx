import { useEffect, useState } from "react";



const SimpleTask = (props) => {

    const [selected, setSelected] = useState(false)
    let bcolor = "w-full h-12  relative border rounded-md flex justify-center items-center " 
    bcolor +=  selected ? 'border-slate-900' : 'border-slate-400'
    useEffect(() => {
        setSelected(props.selected)
    }, [props.selected])
    return (
        <div className={ bcolor} onClick={() => props.selection(props.name)}>
            <p>{props.name}</p>
        </div>
    )
}


export default SimpleTask;
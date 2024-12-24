
import {Link} from 'react-router-dom';
const BarElement = function(props){
    const currentMenu = props.current;
    const cn = currentMenu === props.name ? "bg-slate-600 flex items-center justify-center flex-row w-full h-16 hover:bg-slate-600" : " flex items-center justify-center flex-row w-full h-16 hover:bg-slate-600"
    
    const handleClick = () => {
        props.handle(props.name);
    }

    return (
        <Link to={`/${props.name}`} className={cn} onClick={handleClick}>
            <div className="w-1/4 h-full flex items-center justify-center">
                {props.icon}
            </div>
            <div className="w-3/4  h-full flex items-center justify-start">
               { props.text}
            </div>
        </Link>
    )
}

export default BarElement;
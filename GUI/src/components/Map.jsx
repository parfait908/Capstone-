import React, { useEffect, useRef, useState } from 'react';

import { clientStore } from '../store/clientStore';

const Map = (props) => {
    const imgRef = useRef(null)
    const {mapData} = clientStore()

    useEffect(function(){
        if (mapData) {
            const imageUrl = `data:image/png;base64,${mapData}`
            imgRef.current.src = imageUrl;

            // Clean up the object URL when component is unmounted
            return () => {
                URL.revokeObjectURL(imageUrl);
            };
        }
 
    },[mapData])

    return (
        <div className="w-full h-full ">
            <img  alt="gazebo map output" className="w-full h-full object-cover  " ref={imgRef}/>
        </div>
    );
};

export default Map;

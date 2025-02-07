import React, { useEffect, useRef, useState } from 'react';
import { core } from '../store/core';

const Map = () => {
    const imgRef = useRef(null);
    const { mapData } = core();
    const [loading, setLoading] = useState(true); // Loading state
    const [zoom, setZoom] = useState(1); // Initial zoom level

    const zoomIn = () => {
        setZoom((prevZoom) => Math.min(prevZoom + 0.1, 7)); // Max zoom 7x
    };

    const zoomOut = () => {
        setZoom((prevZoom) => Math.max(prevZoom - 0.1, 1)); // Min zoom 1x (original size)
    };

    const handleWheel = (event) => {
        // Prevent the page from scrolling when zooming
        event.preventDefault();
        
        if (event.deltaY < 0) {
            zoomIn(); // Zoom in when scrolling up
        } else {
            zoomOut(); // Zoom out when scrolling down
        }
    };

    useEffect(() => {
        if (mapData) {
            const imageUrl = `data:image/png;base64,${mapData}`;

            // Set image src only if imgRef.current is available
            imgRef.current.src = imageUrl;
            if(!mapData){
                setLoading(true);
            }
            setLoading(false)
            return () => {
                URL.revokeObjectURL(imageUrl);
            };
        }
    }, [mapData]);

    return (
        <div
            className="w-full h-full relative overflow-hidden"
            onWheel={handleWheel} // Listen for mouse wheel events
        >
            {!mapData ? (
                <div className="absolute inset-0 flex justify-center items-center text-xl">Loading...</div>
            ) : (
                <img
                    alt="gazebo map output"
                    className="transition-transform duration-300"
                    style={{ transform: `scale(${zoom})` }} // Apply zoom with scale
                    ref={imgRef}
                    width="100%"
                    height="100%"
                />
            )}

            {/* Floating zoom-in and zoom-out buttons */}
            <button
                onClick={zoomIn}
                className="w-12 h-12 absolute bottom-10 left-10  bg-blue-500 text-white rounded-full flex items-center justify-center"
            >
                +
            </button>
            <button
                onClick={zoomOut}
                className="w-12 h-12 absolute bottom-10 left-24  bg-red-500 text-white rounded-full flex items-center justify-center"
            >
                -
            </button>
        </div>
    );
};

export default Map;

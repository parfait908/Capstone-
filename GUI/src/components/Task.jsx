import React, { useEffect, useRef, useState } from 'react';

import { clientStore } from '../store/clientStore';

const Task = (props) => {
    const {sendTask, mapping} = clientStore()


    return (
        <div className="w-full h-full px-5 pt-5">
            <button className='w-full  h-12 bg-slate-400  text-white' onClick={mapping}>Mapping</button>
            <label htmlFor="task" className='block mt-10'>Enter a task</label>

            <input className='w-full  h-12 bg-white mt text-black px-5 outline-none' type="text" name="task" id="task" placeholder='e.g a->b->c' />
            <button className='w-full  h-12 bg-slate-400 mt-8 text-white'>Send task</button>
        </div>
    );
};

export default Task;

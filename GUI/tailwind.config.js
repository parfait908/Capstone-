/** @type {import('tailwindcss').Config} */
import daisyui from "daisyui"
export default {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
  ],
  theme: {
    
    extend: {
      height: {
      'calc-100-minus-64': 'calc(100% - 64px)',
      },
      colors: {
        "primary" : "#334155",
        "glass" : "#f3f4f6",
        "background" : "#fb923c",
        "secondary" : "#ffffff",
        "foreground" : "#000000",
      },
      keyframes :{
        sslidein: {
          '0%': { width: '0' },
          '100%': {width: '20%' },
        },
        sslideout: {
          '0%': { width: '20%' },
          '100%': { width: '0' },
        },
        pslidein: {
          '0%': { width: '100%' },
          '100%': {width: '80%' },
        },
        pslideout: {
          '0%': { width: '80%' },
          '100%': { width: '100%' },
        },
      },
      animation: {
        pslidein: 'pslidein 1s forwards',
        pslideout: 'pslideout 1s forwards ',
      },
    },
  },
  plugins: [
    daisyui,
  ],
}

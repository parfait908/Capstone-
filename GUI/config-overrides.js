const path = require('path');
const webpack = require('webpack');

module.exports = function override(config) {
  config.resolve = {
    ...config.resolve,
    alias: {
      ...(config.resolve.alias || {}),
      ros2d: path.resolve(__dirname, 'src/utils/ros2d.js'),
      easeljs: path.resolve(__dirname, 'src/utils/easeljs.js'),
      eventemitter2: path.resolve(__dirname, 'src/utils/eventemitter2.js'),
    },
  };

/*   config.plugins = [
    ...(config.plugins || []),
    new webpack.ProvidePlugin({
      createjs: 'createjs', // EaselJS
      EventEmitter2: 'eventemitter2', // EventEmitter2
    }),
  ]; */

  return config;
};

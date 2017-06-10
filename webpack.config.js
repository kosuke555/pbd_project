const path = require('path');
const webpack = require('webpack');

module.exports = function (env) {
    const plugins = !!env.production ? [ new webpack.optimize.UglifyJsPlugin({ sourceMap: true }) ] : [];
    return {
        entry: './src/index.ts',
        output: {
            filename: 'bundle.js',
            path: path.resolve(__dirname, 'build')
        },
        module: {
            rules: [
                { test: /\.ts$/, use: 'ts-loader' }
            ]
        },
        resolve: {
            extensions: ['.js', '.ts']
        },
        plugins,
        devtool: 'source-map'
    };
};

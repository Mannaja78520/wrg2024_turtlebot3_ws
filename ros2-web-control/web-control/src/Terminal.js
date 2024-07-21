import React, { useRef, useEffect } from 'react';
import { Terminal } from 'xterm';
import 'xterm/css/xterm.css'; // Import xterm CSS

const TerminalComponent = () => {
  const terminalRef = useRef(null);

  useEffect(() => {
    // Create a new terminal instance
    const term = new Terminal();

    // Attach the terminal to the DOM
    term.open(terminalRef.current);

    // Example of using the 'on' method
    term.on('data', data => {
      console.log('Received data:', data);
    });

    // Clean up on unmount
    return () => {
      term.dispose();
    };
  }, []);

  return <div ref={terminalRef} style={{ height: '100%', width: '100%' }} />;
};

export default TerminalComponent;

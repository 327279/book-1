import React, { useState, useEffect } from 'react';

const TextSelector = ({ onTextSelection }) => {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection().toString().trim();
      if (text) {
        setSelectedText(text);
        if (onTextSelection) {
          onTextSelection(text);
        }
      } else {
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, [onTextSelection]);

  return (
    <div className="text-selector-indicator" style={{ display: selectedText ? 'block' : 'none' }}>
      <span className="selected-text-preview">
        Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
      </span>
    </div>
  );
};

export default TextSelector;
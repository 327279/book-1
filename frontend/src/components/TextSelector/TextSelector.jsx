import React, { useState, useEffect } from 'react';
import styles from './TextSelector.module.css';

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

  if (!selectedText) return null;

  return (
    <div className={styles.indicator}>
      <span className={styles.selectedText}>
        {selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}
      </span>
    </div>
  );
};

export default TextSelector;
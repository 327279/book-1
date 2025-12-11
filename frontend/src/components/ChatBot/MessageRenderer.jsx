import React from 'react';

const MessageRenderer = ({ message }) => {
  const { text, sender, sources, confidence, error } = message;

  return (
    <div className={`message-bubble ${sender} ${error ? 'error' : ''}`}>
      <div className="message-text">
        {text}
      </div>
      {sources && sources.length > 0 && (
        <div className="message-sources">
          <small>Sourced from: {sources.map(s => `Ch. ${s.chapter}`).join(', ')}</small>
        </div>
      )}
      {confidence && (
        <div className="message-confidence">
          <small>Confidence: {(confidence * 100).toFixed(1)}%</small>
        </div>
      )}
    </div>
  );
};

export default MessageRenderer;
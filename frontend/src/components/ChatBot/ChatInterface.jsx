import React, { useEffect } from 'react';
import './ChatBot.css';

// Vanilla JS chatbot that gets injected into the page
const ChatInterface = () => {
  useEffect(() => {
    // Only inject once
    if (document.getElementById('custom-chatbot')) return;

    const chatHTML = `
      <div id="custom-chatbot">
        <button id="chat-fab-btn" class="chat-fab" aria-label="Open chat">
          💬
        </button>

        <div id="chat-window" class="chatbot-window" style="display: none;">
          <div class="chatbot-header">
            <div class="chatbot-header-info">
              <span class="chatbot-icon">🤖</span>
              <span class="chatbot-title">AI Assistant</span>
            </div>
            <button id="chat-close-btn" class="chatbot-close">✕</button>
          </div>

          <div id="chat-messages" class="chatbot-messages">
            <div class="chatbot-welcome">
              <p><strong>Hi! I'm your Robotics Assistant.</strong></p>
              <p>Ask me about ROS 2, simulation, NVIDIA Isaac, or anything in the book!</p>
            </div>
          </div>

          <form id="chat-form" class="chatbot-form">
            <input id="chat-input" type="text" placeholder="Ask a question..." />
            <button type="submit">Send</button>
          </form>
        </div>
      </div>
    `;

    // Inject into DOM
    document.body.insertAdjacentHTML('beforeend', chatHTML);

    // Get elements
    const fabBtn = document.getElementById('chat-fab-btn');
    const closeBtn = document.getElementById('chat-close-btn');
    const chatWindow = document.getElementById('chat-window');
    const chatForm = document.getElementById('chat-form');
    const chatInput = document.getElementById('chat-input');
    const messagesDiv = document.getElementById('chat-messages');

    // Demo responses
    const getDemoResponse = (query) => {
      const q = query.toLowerCase();
      if (q.includes('ros')) return "ROS 2 is the robotics middleware framework providing communication infrastructure for robot applications.";
      if (q.includes('simulation') || q.includes('gazebo')) return "Simulation allows testing robots virtually. Gazebo is a physics-based simulator for robotics.";
      if (q.includes('isaac') || q.includes('nvidia')) return "NVIDIA Isaac is a platform for accelerated robotics development with photorealistic simulation.";
      if (q.includes('vla')) return "VLA (Vision-Language-Action) models bridge natural language with robotic actions.";
      return "I'm the Physical AI & Robotics Assistant! Ask me about ROS 2, simulation, NVIDIA Isaac, VLA, or any robotics topic.";
    };

    // Open chat function
    const openChat = (e) => {
      e.stopPropagation();
      e.preventDefault();
      chatWindow.style.display = 'block';
      fabBtn.textContent = '✕';
    };

    // Close chat function
    const closeChat = (e) => {
      e.stopPropagation();
      e.preventDefault();
      chatWindow.style.display = 'none';
      fabBtn.textContent = '💬';
    };

    // Toggle function for FAB
    const toggleChat = (e) => {
      e.stopPropagation();
      e.preventDefault();
      if (chatWindow.style.display === 'none') {
        openChat(e);
      } else {
        closeChat(e);
      }
    };

    // Handle message submission
    const handleSubmit = (e) => {
      e.preventDefault();
      const text = chatInput.value.trim();
      if (!text) return;

      // Add user message
      const userMsg = document.createElement('div');
      userMsg.className = 'chatbot-message user';
      userMsg.textContent = text;
      messagesDiv.appendChild(userMsg);

      // Clear input
      chatInput.value = '';

      // Add bot response
      setTimeout(() => {
        const botMsg = document.createElement('div');
        botMsg.className = 'chatbot-message bot';
        botMsg.textContent = getDemoResponse(text);
        messagesDiv.appendChild(botMsg);
        messagesDiv.scrollTop = messagesDiv.scrollHeight;
      }, 500);
    };

    // Add event listeners
    fabBtn.addEventListener('click', toggleChat);
    closeBtn.addEventListener('click', closeChat);
    chatForm.addEventListener('submit', handleSubmit);

    // Cleanup function
    return () => {
      const chatbot = document.getElementById('custom-chatbot');
      if (chatbot) chatbot.remove();
    };
  }, []);

  return null; // No React rendering needed
};

export default ChatInterface;
import React, { useEffect } from 'react';
import './ChatBot.css';

// Chatbot with global event handlers - v2
const ChatInterface = () => {
  useEffect(() => {
    // Remove any existing chatbot first
    const existingChatbot = document.getElementById('custom-chatbot');
    if (existingChatbot) {
      existingChatbot.remove();
    }

    // Define close function globally so inline onclick can use it
    window.closeChatbot = function () {
      const chatWindow = document.getElementById('chat-window');
      const fabBtn = document.getElementById('chat-fab-btn');
      if (chatWindow) chatWindow.style.display = 'none';
      if (fabBtn) fabBtn.textContent = '💬';
    };

    window.openChatbot = function () {
      const chatWindow = document.getElementById('chat-window');
      const fabBtn = document.getElementById('chat-fab-btn');
      if (chatWindow) chatWindow.style.display = 'block';
      if (fabBtn) fabBtn.textContent = '✕';
    };

    window.toggleChatbot = function () {
      const chatWindow = document.getElementById('chat-window');
      if (!chatWindow) return;
      if (chatWindow.style.display === 'none' || !chatWindow.style.display) {
        window.openChatbot();
      } else {
        window.closeChatbot();
      }
    };

    const chatHTML = `
      <div id="custom-chatbot">
        <button id="chat-fab-btn" class="chat-fab" aria-label="Open chat" onclick="window.toggleChatbot(); return false;">
          💬
        </button>

        <div id="chat-window" class="chatbot-window" style="display: none;">
          <div class="chatbot-header">
            <div class="chatbot-header-info">
              <span class="chatbot-icon">🤖</span>
              <span class="chatbot-title">AI Assistant</span>
            </div>
            <button id="chat-close-btn" type="button" class="chatbot-close" onclick="window.closeChatbot(); return false;">✕</button>
          </div>

          <div id="chat-messages" class="chatbot-messages">
            <div class="chatbot-welcome">
              <p><strong>Hi! I'm your Robotics Assistant.</strong></p>
              <p>Ask me about ROS 2, simulation, NVIDIA Isaac, or anything in the book!</p>
            </div>
          </div>

          <form id="chat-form" class="chatbot-form" onsubmit="event.preventDefault(); window.handleChatSubmit(); return false;">
            <input id="chat-input" type="text" placeholder="Ask a question..." />
            <button type="submit" id="chat-send-btn">Send</button>
          </form>
        </div>
      </div>
    `;

    // Inject into DOM
    document.body.insertAdjacentHTML('beforeend', chatHTML);

    // Get elements after injection
    const chatForm = document.getElementById('chat-form');
    const chatInput = document.getElementById('chat-input');
    const messagesDiv = document.getElementById('chat-messages');
    const fabBtn = document.getElementById('chat-fab-btn');
    const closeBtn = document.getElementById('chat-close-btn');

    // Demo responses
    const getDemoResponse = (query) => {
      const q = query.toLowerCase();
      if (q.includes('ros')) return "ROS 2 is the robotics middleware framework providing communication infrastructure for robot applications.";
      if (q.includes('simulation') || q.includes('gazebo')) return "Simulation allows testing robots virtually. Gazebo is a physics-based simulator for robotics.";
      if (q.includes('isaac') || q.includes('nvidia')) return "NVIDIA Isaac is a platform for accelerated robotics development with photorealistic simulation.";
      if (q.includes('vla')) return "VLA (Vision-Language-Action) models bridge natural language with robotic actions.";
      return "I'm the Physical AI & Robotics Assistant! Ask me about ROS 2, simulation, NVIDIA Isaac, VLA, or any robotics topic.";
    };

    // Handle message submission
    window.handleChatSubmit = function () {
      const chatInput = document.getElementById('chat-input');
      const messagesDiv = document.getElementById('chat-messages');

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

    // Add backup event listeners (in addition to inline onclick)
    if (fabBtn) {
      fabBtn.addEventListener('click', function (e) {
        e.preventDefault();
        e.stopPropagation();
        window.toggleChatbot();
      });
    }

    if (closeBtn) {
      closeBtn.addEventListener('click', function (e) {
        e.preventDefault();
        e.stopPropagation();
        window.closeChatbot();
      });
    }

    if (chatForm) {
      chatForm.addEventListener('submit', function (e) {
        e.preventDefault();
        window.handleChatSubmit();
      });
    }

    // Cleanup function
    return () => {
      const chatbot = document.getElementById('custom-chatbot');
      if (chatbot) chatbot.remove();
      delete window.closeChatbot;
      delete window.openChatbot;
      delete window.toggleChatbot;
      delete window.handleChatSubmit;
    };
  }, []);

  return null; // No React rendering needed
};

export default ChatInterface;
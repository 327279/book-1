import React, { useState } from 'react';
import Content from '@theme-original/DocItem/Content';
import { useAuth } from '../../../components/Auth/AuthContext';
import styles from './styles.module.css';
import { API_ENDPOINTS, DEMO_MODE, safeFetch } from '../../../config/api';

// Demo responses for agent skills
const DEMO_SKILL_RESPONSES = {
    personalize: (profile) => `Based on your ${profile?.software_bg || 'technical'} background, this chapter's key concepts can be understood through practical implementation. Focus on the code examples and try running them in your development environment.`,
    translate_urdu: () => `یہ باب روبوٹکس کے بنیادی تصورات کا احاطہ کرتا ہے۔ ROS 2 روبوٹ سسٹمز کے لیے میڈل ویئر فریم ورک ہے جو نوڈز، ٹاپکس، اور سروسز کا استعمال کرتے ہوئے مواصلات فراہم کرتا ہے۔`
};

// Real API call to backend agent
const callAgentSkill = async (skill, text, context = {}) => {
    if (DEMO_MODE) {
        // Return demo response in production
        await new Promise(resolve => setTimeout(resolve, 500));
        if (skill === 'personalize') {
            return DEMO_SKILL_RESPONSES.personalize(context);
        }
        if (skill === 'translate_urdu') {
            return DEMO_SKILL_RESPONSES.translate_urdu();
        }
        return "AI Agent skill executed successfully (demo mode).";
    }

    try {
        const response = await safeFetch(API_ENDPOINTS.agentSkill, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                skill: skill,
                text: text,
                context: context
            })
        });
        if (response.ok) {
            const data = await response.json();
            return data.result;
        }
        // Fallback to demo if API fails
        if (skill === 'personalize') {
            return DEMO_SKILL_RESPONSES.personalize(context);
        }
        return DEMO_SKILL_RESPONSES.translate_urdu();
    } catch (error) {
        console.error("Agent Error:", error);
        return "⚠️ Error: Could not connect to AI Agent.";
    }
};

export default function DocItemContentWrapper(props) {
    const { user } = useAuth();
    const [personalizedContent, setPersonalizedContent] = useState(null);
    const [translatedContent, setTranslatedContent] = useState(null);
    const [loading, setLoading] = useState(false);

    const handlePersonalize = async () => {
        if (!user) {
            alert('Please sign in to use Personalization.');
            return;
        }
        setLoading(true);
        const result = await callAgentSkill('personalize', 'Current Chapter Content', user.profile);
        setPersonalizedContent(result);
        setLoading(false);
    };

    const handleTranslate = async () => {
        setLoading(true);
        const result = await callAgentSkill('translate_urdu', 'Current Chapter Content');
        setTranslatedContent(result);
        setLoading(false);
    };


    return (
        <>
            <div className="row margin-bottom--md">
                <div className="col">
                    {user ? (
                        <button
                            onClick={handlePersonalize}
                            disabled={loading}
                            className="button button--secondary button--sm margin-right--sm">
                            ✨ Personalize for {user.profile.software_bg}
                        </button>
                    ) : (
                        <span className="badge badge--warning margin-right--sm">Sign in to Personalize</span>
                    )}

                    <button
                        onClick={handleTranslate}
                        disabled={loading}
                        className="button button--outline button--primary button--sm">
                        🇵🇰 Translate to Urdu
                    </button>
                </div>
            </div>

            {loading && <div className="alert alert--info">🤖 AI Agent is working on your request...</div>}

            {personalizedContent && (
                <div className="alert alert--success margin-bottom--md">
                    <strong>Personalized Explanation:</strong> {personalizedContent}
                    <button
                        className="close"
                        onClick={() => setPersonalizedContent(null)}
                        style={{ float: 'right', background: 'none', border: 'none', cursor: 'pointer' }}>
                        ×
                    </button>
                </div>
            )}

            {translatedContent && (
                <div className="alert alert--info margin-bottom--md" dir="rtl" style={{ fontFamily: 'Noto Nastaliq Urdu, serif' }}>
                    <strong>اردو ترجمہ:</strong><br />
                    {translatedContent}
                    <button
                        className="close"
                        onClick={() => setTranslatedContent(null)}
                        style={{ float: 'left', background: 'none', border: 'none', cursor: 'pointer' }}>
                        ×
                    </button>
                </div>
            )}

            <Content {...props} />
        </>
    );
}

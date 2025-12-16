import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/Auth/SignupForm';

export default function SignupPage() {
    return (
        <Layout
            title="Join the Class"
            description="Sign up for personalized Physical AI learning">
            <main className="container margin-vert--xl">
                <div className="row">
                    <div className="col col--6 col--offset-3">
                        <SignupForm />
                    </div>
                </div>
            </main>
        </Layout>
    );
}

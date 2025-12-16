import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function NotFound() {
  return (
    <Layout
      title="Page Not Found"
      description="Page Not Found">
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="hero__title">
              🚧 Robot Lost in Simulation 🚧
            </h1>
            <p className="hero__subtitle">
              We could not find what you were looking for.
            </p>
            <p>
              It seems our path planning algorithm hit a singularity. The page you are looking for might have been moved, deleted, or is currently hallucinating.
            </p>
            <div className="margin-vert--lg">
              <Link
                className="button button--primary button--lg"
                to="/">
                Return to Safety (Home)
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

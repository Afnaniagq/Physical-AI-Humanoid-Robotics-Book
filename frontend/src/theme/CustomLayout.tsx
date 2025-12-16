import React from 'react';
import Layout from '@theme-original/Layout';
import BookTitle from '@site/src/components/BookTitle';
import FeatureButtons from '@site/src/components/FeatureButtons';
import ThematicImage from '@site/src/components/ThematicImage';
import StartCourseButton from '@site/src/components/StartCourseButton';
import './CustomLayout.css';

export default function CustomLayout(props) {
  const {children} = props;
  // This is a naive check. A better approach might be needed depending on the final routing strategy.
  const isHomePage = children.type && children.type.name === 'Home';

  if (!isHomePage) {
    return <Layout {...props}>{children}</Layout>;
  }

  return (
    <Layout {...props}>
      <div className="custom-layout">
        <div className="left-column">
          <BookTitle />
          <FeatureButtons />
          <StartCourseButton />
          {/* Chapter Navigation will go here */}
        </div>
        <div className="right-column">
          <ThematicImage />
        </div>
      </div>
    </Layout>
  );
}
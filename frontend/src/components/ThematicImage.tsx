import React, { useState } from 'react';

export default function ThematicImage() {
  const [imageError, setImageError] = useState(false);

  const handleImageError = () => {
    setImageError(true);
  };

  return (
    <div style={imageError ? { backgroundColor: 'var(--ifm-color-emphasis-300)', width: '100%', height: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center', color: 'var(--ifm-font-color-base-inverse)' } : {}}>
      {imageError ? (
        <span>Image Not Available</span>
      ) : (
        <img
          src="https://via.placeholder.com/600x400"
          alt="Thematic Image"
          onError={handleImageError}
          style={{ maxWidth: '100%', height: 'auto' }}
        />
      )}
    </div>
  );
}